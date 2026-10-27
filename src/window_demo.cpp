#include <glad/gl.h>
#include <GLFW/glfw3.h>
#include <iostream>

void GlfwErrorCallback(int code, const char* description)
{
    std::cerr << "GLFW error " << code << ": " << description << '\n';
}

GLuint CompileShader(GLenum type, const char* source)
{
    GLuint shader = glCreateShader(type);
    if (shader == 0)
    {
        std::cerr << "Failed to create shader\n";
        return 0;
    }

    glShaderSource(shader, 1, &source, nullptr);
    glCompileShader(shader);

    GLint success = GL_FALSE;
    glGetShaderiv(shader, GL_COMPILE_STATUS, &success);

    if (success == GL_FALSE)
    {
        char log[1024]{};
        glGetShaderInfoLog(shader, sizeof(log), nullptr, log);
        std::cerr << "Shader compilation failed:\n" << log << '\n';
        glDeleteShader(shader);
        return 0;
    }

    return shader;
}

int main()
{

    glfwSetErrorCallback(GlfwErrorCallback);

    if (!glfwInit())
    {
        std::cerr << "Failed to initialize GLFW\n";
        return 1;
    }

    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);

    GLFWwindow* window = glfwCreateWindow(640, 480, "Render", nullptr, nullptr);
    if (!window)
    {
        std::cerr << "Failed to create window\n";
        glfwTerminate();
        return 1;
    }

    glfwMakeContextCurrent(window);

    if (gladLoadGL(glfwGetProcAddress) == 0)
    {
        std::cerr << "Failed to load OpenGL functions\n";
        glfwDestroyWindow(window);
        glfwTerminate();
        return 1;
    }

    std::cout << "OpenGL: " << glGetString(GL_VERSION) << '\n';

    const char* vertexSource = R"(
    #version 330 core
    layout(location = 0) in vec3 position;

    void main()
    {
        gl_Position = vec4(position, 1.0);
    }
    )";

    const char* fragmentSource = R"(
    #version 330 core
    out vec4 fragmentColor;

    void main()
    {
        fragmentColor = vec4(0.9, 0.2, 0.1, 1.0);
    }
    )";

    const float vertices[] =
    {
        -0.6f, -0.5f, 0.0f,
        0.6f, -0.5f, 0.0f,
        0.0f,  0.6f, 0.0f
    };

    GLuint vertexShader = CompileShader(GL_VERTEX_SHADER, vertexSource);
    GLuint fragmentShader = CompileShader(GL_FRAGMENT_SHADER, fragmentSource);


    if (vertexShader == 0 || fragmentShader == 0)
    {
        if (vertexShader != 0)
            glDeleteShader(vertexShader);
        if (fragmentShader != 0)
            glDeleteShader(fragmentShader);
        glfwDestroyWindow(window);
        glfwTerminate();
        return 1;
    }

    GLuint program = glCreateProgram();
    if (program == 0)
    {
        std::cerr << "Failed to create shader program\n";
        glDeleteShader(vertexShader);
        glDeleteShader(fragmentShader);
        glfwDestroyWindow(window);
        glfwTerminate();
        return 1;
    }

    glAttachShader(program, vertexShader);
    glAttachShader(program, fragmentShader);
    glLinkProgram(program);

    glDeleteShader(vertexShader);
    glDeleteShader(fragmentShader);


    GLint success = GL_FALSE;
    glGetProgramiv(program, GL_LINK_STATUS, &success);
    if (success == GL_FALSE)
    {
        char log[1024]{};
        glGetProgramInfoLog(program, sizeof(log), nullptr, log);
        std::cerr << "Shader program linking failed:\n" << log << '\n';
        glDeleteProgram(program);
        glfwDestroyWindow(window);
        glfwTerminate();
        return 1;
    }

    GLuint vao = 0;
    GLuint vbo = 0;

    glGenVertexArrays(1, &vao);
    glGenBuffers(1, &vbo);

    glBindVertexArray(vao);

    glBindBuffer(GL_ARRAY_BUFFER, vbo);
    glBufferData(GL_ARRAY_BUFFER, sizeof(vertices), vertices, GL_STATIC_DRAW);

    glVertexAttribPointer(
        0,                  // Номер атрибута
        3,                  // Три компонента: x, y, z
        GL_FLOAT,           // Тип компонента
        GL_FALSE,           // Нормализация не нужна
        3 * sizeof(float),  // Шаг между вершинами в байтах
        nullptr             // Смещение от начала буфера: 0
    );

    glEnableVertexAttribArray(0);

    glBindVertexArray(0);
    glBindBuffer(GL_ARRAY_BUFFER, 0);

    while (!glfwWindowShouldClose(window))
    {
        // Размер буфера в пикселях может отличаться от размера окна.
        int width = 0;
        int height = 0;
        glfwGetFramebufferSize(window, &width, &height);
        glViewport(0, 0, width, height);

        glClearColor(0.72f, 0.79f, 0.74f, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT);

        glUseProgram(program);
        glBindVertexArray(vao);
        glDrawArrays(GL_TRIANGLES, 0, 3);

        glfwSwapBuffers(window);
        glfwPollEvents();

    }

    // OpenGL-объекты освобождаем, пока контекст окна ещё существует.
    glUseProgram(0);
    glBindVertexArray(0);
    glDeleteProgram(program);
    glDeleteVertexArrays(1, &vao);
    glDeleteBuffers(1, &vbo);

    glfwDestroyWindow(window);
    glfwTerminate();
    return 0;
}
