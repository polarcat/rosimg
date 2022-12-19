/* Copyright (C) 2021 Aliaksei Katovich. All rights reserved.
 *
 * This source code is licensed under the BSD Zero Clause License found in
 * the 0BSD file in the root directory of this source tree.
 */

#define LOG_TAG "rosimg"
#include "utils/utils.h"

#define GLAD_GL_IMPLEMENTATION
#include <glad/gl.h>
#define GLFW_INCLUDE_NONE
#include <GLFW/glfw3.h>
#define STB_IMAGE_IMPLEMENTATION
#include <stb/stb_image.h>

#include <stdlib.h>
#include <unistd.h>
#include <stddef.h>
#include <stdio.h>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
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

using topic_info_t = std::vector<rclcpp::TopicEndpointInfo>;
using std::placeholders::_1;
using image_t = sensor_msgs::msg::Image;
using image_ptr = sensor_msgs::msg::Image::SharedPtr;

using zimage_t = sensor_msgs::msg::CompressedImage;
using zimage_ptr = sensor_msgs::msg::CompressedImage::SharedPtr;

#define create_image_subscriber(topic) \
	this->create_subscription<image_t>(topic, 10, \
	 std::bind(&ImageViewer::handle_image, this, _1))

#define create_zimage_subscriber(topic) \
	this->create_subscription<zimage_t>(topic, 10, \
	 std::bind(&ImageViewer::handle_zimage, this, _1))

struct context {
	GLuint prog;
	GLuint vbo;
	GLuint vao;
	GLuint tex;
	GLint u_tex;
	GLint a_pos;
	GLFWwindow *win{nullptr};
	int w{WIN_WIDTH};
	int h{WIN_HEIGHT};
	float aspect_ratio{0};
	float aspect_ratio_reverse{0};
	bool done{false};
	sem_t sem;
	bool print_info{true};
};

enum class image_status : uint8_t {
	free,
	init,
	ready,
	busy,
};

struct image {
	std::atomic<image_status> status;
	image_ptr image{nullptr};
	zimage_ptr zimage{nullptr};
	stbi_uc *zdata{nullptr};
	int zimage_w;
	int zimage_h;
};

static struct image images_[2];

#include "common.cpp"

static void clear_image(struct image *img)
{
	if (img->zimage) {
		img->zimage.reset();
		img->zimage = nullptr;
	}

	if (img->zdata) {
		free(img->zdata);
		img->zdata = nullptr;
	}

	if (img->image) {
		img->image.reset();
		img->image = nullptr;
	}

	img->status = image_status::free;
}

static struct image *find_ready_image(void)
{
	for (uint8_t i = 0; i < ARRAY_SIZE(images_); ++i) {
		if (images_[i].status == image_status::ready)
			return &images_[i];
	}

	return nullptr;
}

static struct image *find_free_image(void)
{
	for (uint8_t i = 0; i < ARRAY_SIZE(images_); ++i) {
		struct image *image = &images_[i];
		if (image->status == image_status::free) {
			image->status = image_status::init;
			return image;
		}
	}

	return nullptr;
}

static void set_aspect_ratio(struct context *ctx, float w, float h)
{
	ctx->aspect_ratio = w / h;
	ctx->aspect_ratio_reverse = h / w;
}

static inline struct image *get_next_image(struct context *ctx)
{
	struct image *img;
	while (!ctx->done) {
		if ((img = find_free_image()))
			return img;
		else
			sem_wait(&ctx->sem); // keep renderer pace..
	}

	return nullptr;
}

static void draw_image(struct context *ctx)
{
	int w;
	int h;
	void *data;

	// tell reader to prepare next image while we are rendering
	sem_post_checked(&ctx->sem);

	struct image *img = find_ready_image();
	if (!img) {
		data = nullptr;
		nop("none of %zu buffers is ready\n", ARRAY_SIZE(images_));
	} else {
		if (img->zimage) {
			w = img->zimage_w;
			h = img->zimage_h;
			data = img->zdata;
		} else {
			w = img->image->width;
			h = img->image->height;
			data = img->image->data.data();
		}
	}

	glUseProgram(ctx->prog);
	glActiveTexture(GL_TEXTURE0);
	glBindTexture(GL_TEXTURE_2D, ctx->tex);

	if (img) {
		glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, w, h, 0, GL_RGB,
		 GL_UNSIGNED_BYTE, data);
		clear_image(img);
	}

	glBindVertexArray(ctx->vao);
        glDrawArrays(GL_TRIANGLES, 0, 6);
}

static void print_image_info(const image_ptr image)
{
	printf("\033[2m");
	printf("| width: %u\n", image->width);
	printf("| height: %u\n", image->height);
	printf("| encoding: %s\n", image->encoding.c_str());
	printf("| be: %u\n", image->is_bigendian);
	printf("| step: %u\n", image->step);
	printf("\033[0m");
}

static bool validate_format(std::string *format)
{
	for (uint8_t i = 0; i < ARRAY_SIZE(zimage_fmt_); ++i) {
		if (format->find(zimage_fmt_[i]) != std::string::npos)
			return true;
	}

	ee("unsupported compressed image format '%s'\n", format->c_str());
	return false;
}

class ImageViewer : public rclcpp::Node
{
public:
	ImageViewer(const char *topic, struct context *ctx);
	~ImageViewer();
private:
	struct context *ctx_;
	void handle_image(const image_ptr);
	rclcpp::Subscription<image_t>::SharedPtr image_{nullptr};
	void handle_zimage(const zimage_ptr);
	rclcpp::Subscription<zimage_t>::SharedPtr zimage_{nullptr};
};

ImageViewer::ImageViewer(const char *topic, struct context *ctx):
 Node(NODE_TAG),
 ctx_(ctx)
{
	// NB: there is some delay before topic information can be retrieved
	// from ROS2; wait some reasonable time
	for (uint8_t i = 0; i < 100; ++i) {
		topic_info_t infos = get_publishers_info_by_topic(topic);
		if (!infos.size()) {
			usleep(100000);
			continue;
		}

		for (auto &info : infos) {
			ii("topic type: %s\n", info.topic_type().c_str());

			if (info.topic_type() == image_msg_) {
				image_ = create_image_subscriber(topic);
				return;
			} else if (info.topic_type() == zimage_msg_) {
				zimage_ = create_zimage_subscriber(topic);
				return;
			}
		}
	}

	ww("failed to get topics, maybe next time, bye!\n");
	exit(1);
}

ImageViewer::~ImageViewer()
{
        ii("stopped image viewer\n");
}

void ImageViewer::handle_image(const image_ptr image)
{
	if (image->encoding != "rgb8" && image->encoding != "8UC3") {
		ee("only rgb8 and 8UC3 images are supported");
		return;
	}

	struct image *img = get_next_image(ctx_);
	if (ctx_->done || !img)
		return;

	if (ctx_->aspect_ratio <= 0)
		set_aspect_ratio(ctx_, image->width, image->height);

	if (ctx_->print_info) {
		print_image_info(image);
		ctx_->print_info = false;
	}

	img->image = image;
	img->status = image_status::ready;
}

void ImageViewer::handle_zimage(const zimage_ptr zimage)
{
	if (!validate_format(&zimage->format))
		return;

	struct image *img = get_next_image(ctx_);
	if (ctx_->done || !img)
		return;

	int n;
	img->zimage = zimage;
	img->zdata = stbi_load_from_memory(zimage->data.data(),
	 zimage->data.size(), &img->zimage_w, &img->zimage_h, &n, 3);

	if (!img->zdata) {
		clear_image(img);
		ee("failed to uncompress image\n");
		return;
	} else if (n != 3) {
		clear_image(img);
		ee("only RGB color scheme is supported\n");
		return;
	} else if (ctx_->aspect_ratio <= 0) {
		set_aspect_ratio(ctx_, img->zimage_w, img->zimage_h);
	}

	if (ctx_->print_info) {
		printf("\033[2m");
		printf("| format: %s\n", zimage->format.c_str());
		printf("\033[0m");
		ctx_->print_info = false;
	}

	img->status = image_status::ready;
}

static void ros_loop(const char *topic, struct context *ctx)
{
	ii("view topic %s\n", topic);
	rclcpp::init(0, NULL);
	rclcpp::spin(std::make_shared<ImageViewer>(topic, ctx));
	ctx->done = true;
	ii("shutdown ros thread\n");
}

int main(int argc, char *argv[])
{
	struct context ctx;
	std::thread loop_thread;
	const char *topic = argv[1];

	if (argc < 2 || !topic) {
		printf("Usage: %s <topic>\n", argv[0]);
		exit(1);
	}

	if (!create_window(&ctx, topic)) {
		ee("failed to init windowing system\n");
		return false;
	}

	for (uint8_t i = 0; i < ARRAY_SIZE(images_); ++i)
		images_[i].status = image_status::free;

	sem_init(&ctx.sem, 0, 0);
	loop_thread = std::thread(ros_loop, topic, &ctx);

	while (!glfwWindowShouldClose(ctx.win) && !ctx.done) {
		/* lame way of tracking window resize */
		glfwGetFramebufferSize(ctx.win, &ctx.w, &ctx.h);

		/* try to maintain original aspect ratio */
		fit_w_ = ctx.h * ctx.aspect_ratio;
		if (fit_w_ <= ctx.w) {
			fit_h_ = ctx.h;
		} else {
			fit_w_ = ctx.w;
			fit_h_ = ctx.w * ctx.aspect_ratio_reverse;
		}

		glViewport(0, 0, fit_w_, fit_h_);
		glClear(GL_COLOR_BUFFER_BIT);
		draw_image(&ctx);
		glfwSwapBuffers(ctx.win);
		glfwPollEvents();
	}

	sem_post_checked(&ctx.sem);
	ctx.done = true;
	glDeleteProgram(ctx.prog);
	glfwDestroyWindow(ctx.win);
	glfwTerminate();

	rclcpp::shutdown(); /* sanity call; destructor should do it first */
	if (loop_thread.joinable())
		loop_thread.join();

	return 0;
}
