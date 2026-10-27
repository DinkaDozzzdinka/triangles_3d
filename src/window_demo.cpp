#include <GLFW/glfw3.h>
#include <system_error>
#include <iostream>

void GlfwErrorCallback(int code, const char* description)
{
    std::cerr << "GLFW error " << code << ": " << description << '\n';
}

int main()
{
    glfwSetErrorCallback(GlfwErrorCallback);
    
    if (!glfwInit())
    {
        std::cerr << "Failed to initialize GLFW\n";
        return 1;
    }

    GLFWwindow* window = glfwCreateWindow(640, 480, "Render", NULL, NULL);
    if (!window)
    {
        std::cerr << "Failed to create window\n";
        glfwTerminate();
        return 1;
    }

    glfwMakeContextCurrent(window);

    while (!glfwWindowShouldClose(window))
    {
        glClearColor(0.72f, 0.79f, 0.74f, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT);
        glfwSwapBuffers(window);
        glfwPollEvents();

    }

    glfwDestroyWindow(window);
    glfwTerminate();
    return 0;
}