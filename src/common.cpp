constexpr uint8_t images_max_ = 2; // image queue depth

constexpr const char *image_msg_ = "sensor_msgs/msg/Image";
constexpr const char *zimage_msg_ = "sensor_msgs/msg/CompressedImage";

constexpr const char *zimage_fmt_[] = {
	"rgb8; jpeg compressed bgr8",
	"jpg",
	"jpeg",
	"JPG",
	"JPEG",
};

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

static int fit_w_;
static int fit_h_;

static void key_cb(GLFWwindow *win, int key, unused_arg(int code), int action,
 unused_arg(int mods))
{
	if (action != GLFW_PRESS)
		return;
	else if (key == GLFW_KEY_ESCAPE || key == GLFW_KEY_Q)
		glfwSetWindowShouldClose(win, GLFW_TRUE);
	else if (key == GLFW_KEY_F)
		glfwSetWindowSize(win, fit_w_, fit_h_);
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

	const float verts[] = {
		-1., 1.,
		1., 1.,
		1., -1.,
		1., -1.,
		-1., -1.,
		-1., 1.,
	};

	glGenBuffers(1, &ctx->vbo);
	glBindBuffer(GL_ARRAY_BUFFER, ctx->vbo);
	glBufferData(GL_ARRAY_BUFFER, sizeof(float) * 2 * 6, verts,
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

	return true;
}

static bool create_window(struct context *ctx, const char *title)
{
	glfwSetErrorCallback(error_cb);
	if (!glfwInit())
		return false;

	glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
	glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
	glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);

	ctx->win = glfwCreateWindow(ctx->w, ctx->h, title, nullptr, nullptr);
	if (!ctx->win) {
		glfwTerminate();
		return false;
	}

	glfwSetKeyCallback(ctx->win, key_cb);
	glfwMakeContextCurrent(ctx->win);
	gladLoadGL(glfwGetProcAddress);
	glfwSwapInterval(1);
	glClearColor(0, 0, 0, 1);

	return make_prog(ctx);
}
