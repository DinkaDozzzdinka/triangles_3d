#include <glad/gl.h>
#include <GLFW/glfw3.h>

#include "bvh.hpp"
#include "input.hpp"
#include "render_data.hpp"

#include <exception>
#include <iostream>
#include <vector>
#include <cstddef>

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
    std::vector<triangles::RenderVertex> render_vertices;
    try
    {
        const auto scene = triangles::ReadScene(std::cin);
        triangles::BvhSearchStats stats{};
        const auto mask = triangles::FindIntersectionsBvh(scene, stats);
        render_vertices = triangles::BuildRenderVertices(scene, mask);
        triangles::FitRenderVerticesToView(render_vertices);
    }
    catch (const std::exception& error)
    {
        std::cerr << "Error: " << error.what() << '\n';
        return 1;
    }

    std::cout << "Prepared render vertices: " << render_vertices.size() << std::endl;

    glfwSetErrorCallback(GlfwErrorCallback);

    if (!glfwInit())
    {
        std::cerr << "Failed to initialize GLFW\n";
        return 1;
    }

    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
    glfwWindowHint(GLFW_DEPTH_BITS, 24);

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

    glEnable(GL_DEPTH_TEST);

    std::cout << "OpenGL: " << glGetString(GL_VERSION) << '\n';

    const char* vertexSource = R"(
    #version 330 core
    layout(location = 0) in vec3 position;
    layout(location = 1) in vec3 color;
    uniform vec2 viewportScale;
    out vec3 vertexColor;

    void main()
    {
        gl_Position = vec4(position.xy * viewportScale, position.z, 1.0);
        vertexColor = color;
    }
    )";

    const char* fragmentSource = R"(
    #version 330 core
    in vec3 vertexColor;
    out vec4 fragmentColor;

    void main()
    {
        fragmentColor = vec4(vertexColor, 1.0);
    }
    )";

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

    const GLint viewport_scale_location = glGetUniformLocation(program, "viewportScale");

    GLuint vao = 0;
    GLuint vbo = 0;

    glGenVertexArrays(1, &vao);
    glGenBuffers(1, &vbo);

    glBindVertexArray(vao);

    glBindBuffer(GL_ARRAY_BUFFER, vbo);
    glBufferData(GL_ARRAY_BUFFER, render_vertices.size() * sizeof(triangles::RenderVertex), render_vertices.data(), GL_STATIC_DRAW);

    glVertexAttribPointer(
        0,                  // Номер атрибута
        3,                  // Три компонента: x, y, z
        GL_DOUBLE,           // Тип компонента
        GL_FALSE,           // Нормализация не нужна
        sizeof(triangles::RenderVertex),  // Шаг между вершинами в байтах
        nullptr             // Смещение от начала буфера: 0
    );

    glEnableVertexAttribArray(0);

    glVertexAttribPointer(
        1,                  // Номер атрибута
        3,                  // Три компонента: x, y, z
        GL_FLOAT,           // Тип компонента
        GL_FALSE,           // Нормализация не нужна
        sizeof(triangles::RenderVertex),  // Шаг между вершинами в байтах
        reinterpret_cast<const void*>(offsetof(triangles::RenderVertex, color))            // Смещение от начала буфера: 0
    );
    glEnableVertexAttribArray(1);

    glBindVertexArray(0);
    glBindBuffer(GL_ARRAY_BUFFER, 0);

    while (!glfwWindowShouldClose(window))
    {
        // Размер буфера в пикселях может отличаться от размера окна.
        int width = 0;
        int height = 0;
        glfwGetFramebufferSize(window, &width, &height);
        if (width == 0 || height == 0)
        {
            glfwWaitEvents();
            continue;
        }
        glViewport(0, 0, width, height);

        glClearColor(0.72f, 0.79f, 0.74f, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        glUseProgram(program);
        const float viewport_width = width;
        const float viewport_height = height;
        float scale_x = 1.0f;
        float scale_y = 1.0f;
        if (width > height)
            scale_x = viewport_height / viewport_width;
        else if (height > width)
            scale_y = viewport_width / viewport_height;
        glUniform2f(viewport_scale_location, scale_x, scale_y);
        glBindVertexArray(vao);
        glDrawArrays(GL_TRIANGLES, 0, render_vertices.size());

        glfwSwapBuffers(window);
        glfwPollEvents();

    }

    glUseProgram(0);
    glBindVertexArray(0);
    glDeleteProgram(program);
    glDeleteVertexArrays(1, &vao);
    glDeleteBuffers(1, &vbo);

    glfwDestroyWindow(window);
    glfwTerminate();
    return 0;
}
