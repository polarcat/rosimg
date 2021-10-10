/* Copyright (C) 2021 Aliaksei Katovich. All rights reserved.
 *
 * This source code is licensed under the BSD Zero Clause License found in
 * the 0BSD file in the root directory of this source tree.
 */

#define GLAD_GL_IMPLEMENTATION
#include <glad/gl.h>
#define GLFW_INCLUDE_NONE
#include <GLFW/glfw3.h>
#define STB_IMAGE_IMPLEMENTATION
#include <stb/stb_image.h>

#include <stdlib.h>
#include <stddef.h>
#include <stdio.h>
#include <semaphore.h>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/compressed_image.hpp"

#ifndef WIN_WIDTH
#define WIN_WIDTH 960
#endif

#ifndef WIN_HEIGHT
#define WIN_HEIGHT 540
#endif

#ifndef NODE_TAG
#define NODE_TAG "imageview"
#endif

#ifndef ARRAY_SIZE
#define ARRAY_SIZE(a) (sizeof(a) / sizeof((a)[0]))
#endif

#define rlog rclcpp::get_logger(NODE_TAG)
#define ii(...) RCLCPP_INFO(rlog, __VA_ARGS__);
#define ww(...) RCLCPP_WARN(rlog, __VA_ARGS__);
#define ee(...) RCLCPP_ERROR(rlog, __VA_ARGS__);
#define nop(...) ;

using std::placeholders::_1;
using image_t = sensor_msgs::msg::CompressedImage;
using image_ptr = sensor_msgs::msg::CompressedImage::SharedPtr;

#define create_subscriber(topic) \
	this->create_subscription<image_t>(topic, 10, \
	 std::bind(&ImageViewer::topic_cb, this, _1))

namespace {

static const char *vsrc_ =
	"#version 330\n"
	"in vec2 a_pos;\n"
	"out vec2 v_uv;\n"
	"void main(){\n"
		"float x=float(((uint(gl_VertexID)+2u)/3u)%2u);\n"
		"float y=float(((uint(gl_VertexID)+1u)/3u)%2u);\n"
		"gl_Position=vec4(a_pos,0.,1.);\n"
		"v_uv=vec2(x,y);\n"
	"}\n";

static const char *fsrc_ =
	"#version 330\n"
	"uniform sampler2D u_tex;\n"
	"in vec2 v_uv;\n"
	"out vec4 frag;\n"
	"void main(){\n"
		"frag=texture2D(u_tex,v_uv);\n"
	"}\n";

static const float verts_[] = {
	-1., 1.,
	1., 1.,
	1., -1.,
	1., -1.,
	-1., -1.,
	-1., 1.,
};

struct context {
	GLuint prog;
	GLuint vbo;
	GLuint vao;
	GLuint tex;
	GLint u_tex;
	GLint a_pos;
	uint32_t dropped_frames;
	float ratio;
	uint32_t sec;
	uint32_t nsec;
	float fps;
};

enum {
	STATE_NONE,
	STATE_READY,
	STATE_DRAW,
};

struct image {
	uint8_t state;
	stbi_uc *data;
	int w;
	int h;
	uint32_t id;
	uint32_t prev_id;
	uint32_t sec;
	uint32_t nsec;
	pthread_mutex_t lock;
};

static struct image images_[2];

void calc_fps(struct context *ctx, struct image *img)
{
	uint64_t ms1 = ctx->sec * 1000 + ctx->nsec * .000001;
	uint64_t ms2 = img->sec * 1000 + img->nsec * .000001;
	ctx->fps = 1. / ((ms2 - ms1) * .001);
	ctx->sec = img->sec;
	ctx->nsec = img->nsec;
	ii("fps %.1f", ctx->fps);
}

} /* namespace */

class ImageViewer : public rclcpp::Node
{
public:
	ImageViewer(const char *topic);
	~ImageViewer();
private:
	void topic_cb(const image_ptr);
	rclcpp::Subscription<image_t>::SharedPtr viewer_;
	uint32_t counter_ = 0;
};

void ImageViewer::topic_cb(const image_ptr msg)
{
	int w;
	int h;
	int n;
	/* TODO: decompress into pre-allocated buffer */
	stbi_uc *buf = stbi_load_from_memory(msg->data.data(), msg->data.size(),
	 &w, &h, &n, 3);
	nop("frame: %u, format: %s, buf: %p, wh(%d %d), planes: %d", ++counter_,
	 msg->format.c_str(), buf, w, h, n);

	if (n != 3) /* TODO: support other formats */
		return;

	/* TODO: track frame drops; calc FPS */
	for (uint8_t i = 0; i < ARRAY_SIZE(images_); ++i) {
		struct image *image = &images_[i];
		pthread_mutex_lock(&image->lock);
		uint8_t state = image->state;
		pthread_mutex_unlock(&image->lock);

		if (state != STATE_NONE) {
			continue;
		} else {
			image->sec = msg->header.stamp.sec;
			image->nsec = msg->header.stamp.nanosec;
			image->id++;
			image->data = buf;
			image->w = w;
			image->h = h;
			image->state = STATE_READY;
			break;
		}
	}
}

ImageViewer::~ImageViewer()
{
        ii("Stopped image viewer");
}

ImageViewer::ImageViewer(const char *topic): Node(NODE_TAG)
{
	viewer_ = create_subscriber(topic);
}

static void key_cb(GLFWwindow *win, int key, int code, int action, int mods)
{
	if (key == GLFW_KEY_ESCAPE && action == GLFW_PRESS)
		glfwSetWindowShouldClose(win, GLFW_TRUE);
}

static void error_cb(int err, const char *str)
{
	fprintf(stderr, "%s, err=%d\n", str, err);
}

static inline bool gl_error(const char *msg)
{
	bool ret = false;
	for (GLint err = glGetError(); err; err = glGetError()) {
		ee("%s error '0x%x'", msg, err);
		ret = true;
	}
	return ret;
}

static GLuint make_shader(GLenum type, const char *src)
{
	GLuint shader = glCreateShader(type);

	if (!shader) {
		gl_error("create shader");
		return 0;
	}

	glShaderSource(shader, 1, &src, NULL);
	glCompileShader(shader);
	GLint compiled = 0;
	glGetShaderiv(shader, GL_COMPILE_STATUS, &compiled);

	if (!compiled) {
		GLint len = 0;
		glGetShaderiv(shader, GL_INFO_LOG_LENGTH, &len);

		if (!len)
			return 0;

		char *buf = (char *) malloc(len);
		if (!buf)
			return 0;

		errno = 0;
		glGetShaderInfoLog(shader, len, NULL, buf);
		ee("could not compile %s shader: %s",
		 type == GL_VERTEX_SHADER ? "vertex" : "fragment", buf);
		free(buf);
		glDeleteShader(shader);
		return 0;
	}

	return shader;
}

static bool make_prog(struct context *ctx)
{
	ctx->prog = glCreateProgram();
	if (!ctx->prog) {
		gl_error("create program");
		return false;
	}

	GLuint vsh = make_shader(GL_VERTEX_SHADER, vsrc_);
	if (!vsh)
		return false;

	GLuint fsh = make_shader(GL_FRAGMENT_SHADER, fsrc_);
	if (!fsh)
		return false;

	glAttachShader(ctx->prog, vsh);
	glAttachShader(ctx->prog, fsh);
	glLinkProgram(ctx->prog);

	GLint status = GL_FALSE;
	glGetProgramiv(ctx->prog, GL_LINK_STATUS, &status);
	if (status != GL_TRUE) {
		GLint len = 0;
		glGetProgramiv(ctx->prog, GL_INFO_LOG_LENGTH, &len);

		if (len) {
			char *buf = (char *) malloc(len);

			if (buf) {
				errno = 0;
				glGetProgramInfoLog(ctx->prog, len, NULL, buf);
				ee("%s", buf);
				free(buf);
			}
		}

		glDeleteProgram(ctx->prog);
		ee("failed to link program %u", ctx->prog);
		ctx->prog = 0;
		return false;
	}

	ctx->u_tex = glGetUniformLocation(ctx->prog, "u_tex");
	ctx->a_pos = glGetAttribLocation(ctx->prog, "a_pos");

	glGenBuffers(1, &ctx->vbo);
	glBindBuffer(GL_ARRAY_BUFFER, ctx->vbo);
	glBufferData(GL_ARRAY_BUFFER, sizeof(float) * 2 * 6, verts_,
	 GL_STATIC_DRAW);
	/* keep VBO bound for VAO */

	glGenVertexArrays(1, &ctx->vao);
	glBindVertexArray(ctx->vao);
	glEnableVertexAttribArray(ctx->a_pos);
	glVertexAttribPointer(ctx->a_pos, 2, GL_FLOAT, GL_FALSE, 0, 0);

	glBindVertexArray(0);
	glBindBuffer(GL_ARRAY_BUFFER, 0);

	glGenTextures(1, &ctx->tex);
	glBindTexture(GL_TEXTURE_2D, ctx->tex);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_BORDER);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_BORDER);

	ctx->dropped_frames = 0;
	return true;
}

static void draw_image(struct context *ctx)
{
	struct image *image;

	glUseProgram(ctx->prog);
	glActiveTexture(GL_TEXTURE0);
	glBindTexture(GL_TEXTURE_2D, ctx->tex);

	for (uint8_t i = 0; i < ARRAY_SIZE(images_); ++i) {
		bool draw = false;
		image = &images_[i];
		pthread_mutex_lock(&image->lock);
		if (image->state == STATE_READY) {
			image->state = STATE_DRAW;
			draw = true;
		}
		pthread_mutex_unlock(&image->lock);
		if (draw)
			break;
		else
			image = NULL;
	}

	if (image) {
		glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, image->w, image->h, 0,
		 GL_RGB, GL_UNSIGNED_BYTE, image->data);
		ctx->ratio = (float) image->w / image->h;
		pthread_mutex_lock(&image->lock);
		int32_t diff = image->id - image->prev_id;
		if (diff == 0) {
			ww("repeat frame %u", image->id);
		} else if (diff > 1) {
			ctx->dropped_frames++;
			ww("dropped %u frames", ctx->dropped_frames);
		}
		image->prev_id = image->id;
		free(image->data);
		image->data = NULL;
		image->state = STATE_NONE;
#ifdef PRINT_FPS
		calc_fps(ctx, image);
#endif
		pthread_mutex_unlock(&image->lock);
	}

	glBindVertexArray(ctx->vao);
        glDrawArrays(GL_TRIANGLES, 0, 6);
}

void ros_loop(const char *topic, bool *done)
{
	ii("View topic %s", topic);
	rclcpp::init(0, NULL);
	rclcpp::spin(std::make_shared<ImageViewer>(topic));
	*done = true;
	ii("Shutdown ros thread");
}

int main(int argc, char *argv[])
{
	bool done;
	int w = WIN_WIDTH;
	int h = WIN_HEIGHT;
	struct context ctx;
	GLFWwindow *win;
	std::thread ros;
	const char *topic = argv[1];

	if (argc < 2 || !topic) {
		printf("Usage: %s <topic>\n", argv[0]);
		exit(1);
	}

	for (uint8_t i = 0; i < ARRAY_SIZE(images_); ++i)
		pthread_mutex_init(&images_[i].lock, NULL);

	glfwSetErrorCallback(error_cb);
	if (!glfwInit())
		exit(1);

	glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
	glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
	glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);

	if (!(win = glfwCreateWindow(w, h, topic, NULL, NULL))) {
		glfwTerminate();
		exit(1);
	}

	glfwSetKeyCallback(win, key_cb);
	glfwMakeContextCurrent(win);
	gladLoadGL(glfwGetProcAddress);
	glfwSwapInterval(1);

	if (!make_prog(&ctx))
		exit(1);

	done = false;
	ros = std::thread(ros_loop, topic, &done);

	ctx.ratio = (float) w / h;
	while (!glfwWindowShouldClose(win) && !done) {
		/* lame way of tracking window resize */
		glfwGetFramebufferSize(win, &w, &h);
		glViewport(0, 0, h * ctx.ratio, h);
		draw_image(&ctx);
		glfwSwapBuffers(win);
		glfwPollEvents();
	}

	glDeleteProgram(ctx.prog);
	glfwDestroyWindow(win);
	glfwTerminate();

	rclcpp::shutdown(); /* sanity call; destructor should do it first */
	if (ros.joinable())
		ros.join();

	return 0;
}
