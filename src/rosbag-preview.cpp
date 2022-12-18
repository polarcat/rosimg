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
#include <stddef.h>
#include <stdio.h>

#include <rosbag2_cpp/readers/sequential_reader.hpp>
#include <rosbag2_cpp/typesupport_helpers.hpp>
#include <rosbag2_cpp/converter.hpp>
#ifdef ROS_DISTRO_FOXY
#include <rosbag2_cpp/storage_options.hpp>
using storage_options_t = rosbag2_cpp::StorageOptions;
#else
#include <rosbag2_storage/storage_options.hpp>
using storage_options_t = rosbag2_storage::StorageOptions;
#endif
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>

#ifndef WIN_WIDTH
#define WIN_WIDTH 960
#endif

#ifndef WIN_HEIGHT
#define WIN_HEIGHT 540
#endif

constexpr uint8_t images_max_ = 2; // image queue depth

constexpr const char *storage_id_ = "sqlite3";
constexpr const char *format_ = "cdr";
constexpr const char *roscpp_ = "rosidl_typesupport_cpp";

constexpr const char *image_msg_ = "sensor_msgs/msg/Image";
constexpr const char *zimage_msg_ = "sensor_msgs/msg/CompressedImage";
constexpr const char *zimage_fmt_ = "rgb8; jpeg compressed bgr8";

using namespace rosbag2_cpp::converter_interfaces;
using namespace rosbag2_cpp;

struct reader {
	readers::SequentialReader io;
	std::shared_ptr<rosbag2_introspection_message_t> msg{nullptr};
	std::shared_ptr<rcpputils::SharedLibrary> idl_lib{nullptr};
	const rosidl_message_type_support_t *idl_type{nullptr};
	std::shared_ptr<rosbag2_storage::SerializedBagMessage> ser{nullptr};
	std::unique_ptr<SerializationFormatDeserializer> des{nullptr};

	std::string type_name;

	float aspect_ratio{0};
	float aspect_ratio_reverse{0};
	sem_t sem;
	bool print_info{true};
	bool done{false};
};

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
	struct reader *reader{nullptr};
};

enum class image_status : uint8_t {
	free,
	init,
	ready,
	busy,
};

using image_ptr = std::unique_ptr<sensor_msgs::msg::Image>;
using zimage_ptr = std::unique_ptr<sensor_msgs::msg::CompressedImage>;

struct image {
	std::atomic<image_status> status;
	image_ptr image{nullptr};
	zimage_ptr zimage{nullptr};
	stbi_uc *zdata{nullptr};
	int zimage_w;
	int zimage_h;
};

static struct image images_[images_max_];

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

static void draw_image(struct context *ctx)
{
	int w;
	int h;
	void *data;

	// tell reader to prepare next image while we are rendering
	sem_post_checked(&ctx->reader->sem);

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

static void clear_reader(struct reader *reader)
{
	reader->msg.reset();
	reader->msg = nullptr;

	reader->idl_lib.reset();
	reader->idl_lib = nullptr;

	reader->ser.reset();
	reader->ser = nullptr;

	reader->des.reset();
	reader->des = nullptr;

	reader->idl_type = nullptr;
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

static void set_aspect_ratio(struct reader *reader, float w, float h)
{
	reader->aspect_ratio = w / h;
	reader->aspect_ratio_reverse = h / w;
}

static inline struct image *get_next_image(struct reader *reader)
{
	struct image *img;
	while (!reader->done) {
		if ((img = find_free_image()))
			return img;
		else
			sem_wait(&reader->sem); // keep renderer pace..
	}

	return nullptr;
}

static void print_image_info(struct image *img)
{
	printf("\033[2m");
	printf("| width: %u\n", img->image->width);
	printf("| height: %u\n", img->image->height);
	printf("| encoding: %s\n", img->image->encoding.c_str());
	printf("| be: %u\n", img->image->is_bigendian);
	printf("| step: %u\n", img->image->step);
	printf("\033[0m");
}

static bool handle_image(struct reader *reader)
{
	struct image *img = get_next_image(reader);
	if (reader->done || !img)
		return false;

	img->image = std::make_unique<sensor_msgs::msg::Image>();
	if (!img->image) {
		ee("failed to allocate image memory\n");
		img->status = image_status::free;
		return false;
	}

	reader->msg->message = img->image.get();
	reader->des->deserialize(reader->ser, reader->idl_type, reader->msg);

	if (reader->print_info) {
		print_image_info(img);
		reader->print_info = false;
	}

	if (reader->aspect_ratio <= 0)
		set_aspect_ratio(reader, img->image->width, img->image->height);

	if (img->image->encoding == "rgb8" || img->image->encoding == "8UC3") {
		img->status = image_status::ready;
	} else {
		ee("only rgb8 and 8UC3 images are supported");
		clear_image(img);
		return false;
	}

	return true;
}

static bool uncompress(struct reader *reader, struct image *img)
{
	int n;
	img->zdata = stbi_load_from_memory(img->zimage->data.data(),
	 img->zimage->data.size(), &img->zimage_w, &img->zimage_h, &n, 3);

	if (!img->zdata) {
		ee("failed to uncompress image\n");
		return false;
	} else if (n != 3) {
		ee("only RGB color scheme is supported\n");
		return false;
	} else if (reader->aspect_ratio <= 0) {
		set_aspect_ratio(reader, img->zimage_w, img->zimage_h);
	}

	img->status = image_status::ready;
	return true;
}

static bool handle_zimage(struct reader *reader)
{
	struct image *img = get_next_image(reader);
	if (reader->done || !img)
		return false;

	img->zimage = std::make_unique<sensor_msgs::msg::CompressedImage>();
	if (!img->zimage) {
		ee("failed to allocate image memory\n");
		img->status = image_status::free;
		return false;
	}

	reader->msg->message = img->zimage.get();
	reader->des->deserialize(reader->ser, reader->idl_type, reader->msg);

	if (reader->print_info) {
		printf("\033[2m");
		printf("| format: %s\n", img->zimage->format.c_str());
		printf("\033[0m");
		reader->print_info = false;
	}

	if (img->zimage->format == zimage_fmt_) {
		return uncompress(reader, img);
	} else {
		ee("only jpeg images are supported\n");
		clear_image(img);
		return false;
	}

	return true;
}

static void rosbag_loop(struct reader *reader, const char *topic)
{
	ii("view topic %s\n", topic);
	reader->msg->allocator = rcutils_get_default_allocator();

	while (reader->io.has_next()) {
		reader->msg->time_stamp = 0;
		reader->msg->message = nullptr;

		if (!(reader->ser = reader->io.read_next())) {
			continue;
		} else if (reader->ser->topic_name == topic) {
			if (reader->type_name == image_msg_) {
				if (!handle_image(reader))
					break;
			} else if (reader->type_name == zimage_msg_) {
				if (!handle_zimage(reader))
					break;
			}
		}
	}

	clear_reader(reader);
	reader->done = true;
	ii("shutdown rosbag thread\n");
}

static void list_topics(struct reader *reader)
{
	auto topics = reader->io.get_all_topics_and_types();
	for (auto &t : topics) {
		ii("topic\n");
		ii("\tname: %s\n", t.name.c_str());
		ii("\ttype: %s\n", t.type.c_str());
		ii("\tformat: %s\n", t.serialization_format.c_str());

		if (t.serialization_format != format_) {
			ee("unsupported format '%s'\n",
			 t.serialization_format.c_str());
			continue;
		}
	}
}

static bool validate_topic_type(struct reader *reader, const char *type)
{
	ii("get type support library for '%s'\n", type);
	reader->idl_lib = get_typesupport_library(type, roscpp_);
	if (!reader->idl_lib) {
		ee("failed to get type support library for '%s'\n", type);
		return false;
	}

	ii("get type support handle for '%s'\n", type);
	reader->idl_type = get_typesupport_handle(type, roscpp_,
	 reader->idl_lib);
	if (!reader->idl_type) {
		ee("type '%s' not supported\n", type);
		return false;
	}

	reader->type_name = type;
	return true;
}

static bool validate_topic(struct reader *reader, const char *name)
{
	auto topics = reader->io.get_all_topics_and_types();
	for (auto &t : topics) {
		if (t.name == name)
			return validate_topic_type(reader, t.type.c_str());
	}

	return false;
}

static bool open_rosbag(struct reader *reader, const char *uri)
{
	storage_options_t storage_options{};
	storage_options.uri = uri;
	storage_options.storage_id = storage_id_;

	ConverterOptions converter_options{};
	converter_options.input_serialization_format = format_;
	converter_options.output_serialization_format = format_;

	reader->io.open(storage_options, converter_options);

	SerializationFormatConverterFactory factory;
	reader->des = factory.load_deserializer(format_);
	if (!reader->des) {
		ee("failed to load deserializer for '%s'\n", format_);
		return false;
	}

	return true;
}

int main(int argc, char *argv[])
{
	struct context ctx;
	std::thread loop_thread;
	const char *topic;
	struct reader reader;

	if (argc < 2) {
		printf("Usage: %s <rosbag> [<topic>]\n", argv[0]);
		printf("Examples:\n\033[2m");
		printf("| list topics: %s rosbag.db3\n", argv[0]);
		printf("| play topic: %s rosbag.db3 /image_raw\n", argv[0]);
		printf("\033[0m\n");
		return 1;
	} else if (open_rosbag(&reader, argv[1])) {
		if (argc == 2) {
			list_topics(&reader);
			exit(0);
		}
	}

	if (!validate_topic(&reader, argv[2]))
		return 1;

	topic = argv[2];
	reader.msg = std::make_shared<rosbag2_introspection_message_t>();
	if (!reader.msg) {
		ee("failed to allocate message memory\n");
		return false;
	} else if (!create_window(&ctx, topic)) {
		ee("failed to init windowing system\n");
		return false;
	}

	for (uint8_t i = 0; i < ARRAY_SIZE(images_); ++i)
		images_[i].status = image_status::free;

	ctx.reader = &reader;
	sem_init(&reader.sem, 0, 0);
	loop_thread = std::thread(rosbag_loop, &reader, topic);

	while (!glfwWindowShouldClose(ctx.win) && !reader.done) {
		/* lame way of tracking window resize */
		glfwGetFramebufferSize(ctx.win, &ctx.w, &ctx.h);

		/* try to maintain original aspect ratio */
		fit_w_ = ctx.h * reader.aspect_ratio;
		if (fit_w_ <= ctx.w) {
			fit_h_ = ctx.h;
		} else {
			fit_w_ = ctx.w;
			fit_h_ = ctx.w * reader.aspect_ratio_reverse;
		}

		glViewport(0, 0, fit_w_, fit_h_);
		glClear(GL_COLOR_BUFFER_BIT);
		draw_image(&ctx);
		glfwSwapBuffers(ctx.win);
		glfwPollEvents();
	}

	sem_post_checked(&reader.sem);
	reader.done = true;
	glDeleteProgram(ctx.prog);
	glfwDestroyWindow(ctx.win);
	glfwTerminate();

	if (loop_thread.joinable())
		loop_thread.join();

	return 0;
}
