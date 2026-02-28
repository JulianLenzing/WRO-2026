#ifndef LINE_QUALITY_GRAPHICSFULL_H
#define LINE_QUALITY_GRAPHICSFULL_H

#include "include/glad/glad.h"
#include <glfw3.h>
#include <iostream>
#include <vector>

#include "Vec2f.h"
#include "Line.h"
#include "DisplayData.h"
#include "RGBA.h"

using namespace std;

class Graphics {
    public:
    // Shader sources
    const char* vertexShaderSource = R"glsl(
    #version 330 core
    layout(location = 0) in vec2 aPos;     // vertex position
    layout(location = 1) in vec4 aColor;   // vertex color

    out vec4 vColor;   // pass to fragment shader

    void main() {
        gl_Position = vec4(aPos, 0.0, 1.0);
        vColor = aColor;
    }
    )glsl";

    const char* fragmentShaderSource = R"glsl(
    #version 330 core

    in vec4 vColor;     // interpolated color
    out vec4 FragColor;

    void main() {
        FragColor = vColor;
    }
    )glsl";

    // Window size
    int windowWidth {1000};
    int windowHeight {1000};

    GLFWwindow* window;
    RGBA backgroundColor{};
    unsigned int linesVAO{}, linesVBO{}, pointsVAO{}, pointsVBO{};
    unsigned int shaderProgram{};
    const int VERTEX_SIZE{6};

    explicit Graphics(const int width = 1000, const int height = 1000, RGBA pBackgroundColor = RGBA()) {
        windowWidth = width;
        windowHeight = height;
        backgroundColor = pBackgroundColor;

        window = init_window(windowWidth, windowHeight);
        glfwMakeContextCurrent(window);

        generate_shaderProgram(shaderProgram, vertexShaderSource, fragmentShaderSource);

        // Create VAO/VBO
        glGenVertexArrays(1, &linesVAO);
        glGenVertexArrays(1, &pointsVAO);
        glGenBuffers(1, &linesVBO);
        glGenBuffers(1, &pointsVBO);

        glBindVertexArray(linesVAO);
        glBindBuffer(GL_ARRAY_BUFFER, linesVBO);
        glBufferData(GL_ARRAY_BUFFER, 1024 * sizeof(float), nullptr, GL_DYNAMIC_DRAW);
        // ---- vertex attrib 0: position (vec2) ----
        glVertexAttribPointer(
            0,                                    // location = 0
            2,                                    // vec2
            GL_FLOAT,
            GL_FALSE,
            6 * sizeof(float),                    // stride = 6 floats
            (void*)0                              // offset = 0
        );
        glEnableVertexAttribArray(0);

        // ---- vertex attrib 1: color (vec4) ----
        glVertexAttribPointer(
            1,                                    // location = 1
            4,                                    // vec4
            GL_FLOAT,
            GL_FALSE,
            6 * sizeof(float),                    // stride = 6 floats
            (void*)(2 * sizeof(float))            // offset after x,y
        );
        glEnableVertexAttribArray(1);
        glBindBuffer(GL_ARRAY_BUFFER, 0);
        glBindVertexArray(0);

        glBindVertexArray(pointsVAO);
        glBindBuffer(GL_ARRAY_BUFFER, pointsVBO);
        glBufferData(GL_ARRAY_BUFFER, 1024 * sizeof(float), nullptr, GL_DYNAMIC_DRAW);
        // position
        glVertexAttribPointer(
            0, 2, GL_FLOAT, GL_FALSE,
            6 * sizeof(float),
            (void*)0
        );
        glEnableVertexAttribArray(0);

        // color
        glVertexAttribPointer(
            1, 4, GL_FLOAT, GL_FALSE,
            6 * sizeof(float),
            (void*)(2 * sizeof(float))
        );
        glEnableVertexAttribArray(1);
        glBindBuffer(GL_ARRAY_BUFFER, 0);
        glBindVertexArray(0);

        glPointSize(10.0f);
    }

    // Initialize GLFW, create window and load GLAD
    static GLFWwindow* init_window(int width, int height) {
        GLFWwindow* window = glfwCreateWindow(width, height, "OpenGL Lines", nullptr, nullptr);
        if (!window) {
            std::cerr << "Failed to create GLFW window\n";
            glfwTerminate();
            throw std::runtime_error("Graphics: window creation failed");
        }

        glfwMakeContextCurrent(window);

        if (!gladLoadGLLoader((GLADloadproc)glfwGetProcAddress)) {
            std::cerr << "Failed to initialize GLAD\n";
            throw std::runtime_error("Graphics: window creation failed");
        }

        // Set viewport
        glViewport(0, 0, width, height);

        return window;
    }

    static void generate_shaderProgram(unsigned int& shaderProgram, const char* vertexShaderSource, const char* fragmentShaderSource) {
        // Compile shaders
        unsigned int vertexShader = glCreateShader(GL_VERTEX_SHADER);
        glShaderSource(vertexShader, 1, &vertexShaderSource, nullptr);
        glCompileShader(vertexShader);

        unsigned int fragmentShader = glCreateShader(GL_FRAGMENT_SHADER);
        glShaderSource(fragmentShader, 1, &fragmentShaderSource, nullptr);
        glCompileShader(fragmentShader);

        shaderProgram = glCreateProgram();
        glAttachShader(shaderProgram, vertexShader);
        glAttachShader(shaderProgram, fragmentShader);
        glLinkProgram(shaderProgram);

        int success;
        char infoLog[512];
        glGetShaderiv(vertexShader, GL_COMPILE_STATUS, &success);
        if (!success) {
            glGetShaderInfoLog(vertexShader, 512, NULL, infoLog);
            std::cerr << "Vertex shader compile error:\n" << infoLog << '\n';
        }
        glGetProgramiv(shaderProgram, GL_LINK_STATUS, &success);
        if (!success) {
            glGetProgramInfoLog(shaderProgram, 512, NULL, infoLog);
            std::cerr << "Shader program link error:\n" << infoLog << '\n';
        }
        glDeleteShader(vertexShader);
        glDeleteShader(fragmentShader);
    }

    static void normalisePoints(vector<Vec2f>& points, float maxVal = 4.0f) {
        for (auto& p : points) {
            p.x /= maxVal / 2.0f;
            p.y /= maxVal / 2.0f;
            p.x -= 0.75;
            p.y -= 0.75;
        }
    }

    static void appendPointVertex(std::vector<float>& buffer, const Vec2f& p, const RGBA& color = RGBA()) {
        buffer.push_back(p.x);
        buffer.push_back(p.y);
        buffer.push_back(color.r);
        buffer.push_back(color.g);
        buffer.push_back(color.b);
        buffer.push_back(color.a);
    }

    void drawVertices(GLuint VAO, GLuint VBO, const std::vector<float>& vertices, GLenum mode) const {
        glfwMakeContextCurrent(window);
        glBindVertexArray(VAO);
        glBindBuffer(GL_ARRAY_BUFFER, VBO);

        glBufferData(
            GL_ARRAY_BUFFER,
            vertices.size() * sizeof(float),
            vertices.data(),
            GL_DYNAMIC_DRAW
        );

        GLsizei vertexCount = vertices.size() / VERTEX_SIZE;
        glDrawArrays(mode, 0, vertexCount);

        glBindVertexArray(0);
    }

    void update(const vector<Line>& lines, const vector<RGBA> &lineColors, vector<Vec2f> points, const vector<RGBA>& pointColors) const {
        glfwMakeContextCurrent(window);

        vector<float> pointVertices;
        vector<float> lineVertices;

        normalisePoints(points);
        for (int i = 0; i < points.size(); i++) {
            appendPointVertex(pointVertices, points[i], pointColors[i]);
        }

        vector<Vec2f> linePoints;
        for (const auto& l : lines) {
            linePoints.push_back(l.start);
            linePoints.push_back(l.end);
        }
        normalisePoints(linePoints);
        for (int i = 0; i < linePoints.size(); i++) {
            appendPointVertex(lineVertices, linePoints[i], lineColors[i]);
        }

        glClearColor(backgroundColor.r,backgroundColor.g,backgroundColor.b,backgroundColor.a);
        glClear(GL_COLOR_BUFFER_BIT);
        glUseProgram(shaderProgram);

        drawVertices(linesVAO, linesVBO, lineVertices, GL_LINES);
        drawVertices(pointsVAO, pointsVBO, pointVertices, GL_POINTS);

        glfwSwapBuffers(window);
        glfwPollEvents();
    }

    void update(const DisplayData& data) {
        update(data.getDisplayLines(), data.getDisplayLinesColors(), data.getDisplayPoints(), data.getDisplayPointsColors());
    }

};


#endif //LINE_QUALITY_GRAPHICSFULL_H
