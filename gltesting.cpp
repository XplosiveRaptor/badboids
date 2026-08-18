#include <cstdio>
#include <cstdlib>
#include <GL/glew.h>
#include <GLFW/glfw3.h>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <vector>
#include <random>

using namespace std;
using namespace glm;

constexpr int W_WIDTH = 1000;
constexpr int W_HEIGHT = 1000;

constexpr int NUM_BOIDS = 1000;
constexpr float BOID_SIZE = 0.01f;

constexpr float BOID_VIEW = 0.3f;
constexpr float BOID_CLOSE = 0.1f;

constexpr float ALIGNMENT = 0.01f;
constexpr float COHESION = 0.01f;
constexpr float SEPARATION = 0.04f;

constexpr float TWO_PI = 6.28318531f;
constexpr float PI = 3.14159265f;
constexpr float HALF_PI = 1.57079633f;

constexpr vec3 hue2rgb(float hue)
{
    vec3 result = {0, 0, 0};
    if (hue >= 360.0f)
        hue = 0.0f;
    hue /= 60.0f;
    int i = static_cast<int>(hue);
    float f = hue - i;
    float q = 1.0f - f;
    switch (i)
    {
    case 0:
        result = {1.0f, f, 0};
        break;
    case 1:
        result = {q, 1.0f, 0};
        break;
    case 2:
        result = {0, 1.0f, f};
        break;
    case 3:
        result = {0, q, 1.0f};
        break;
    case 4:
        result = {f, 0, 1.0f};
        break;
    default:
        result = {1.0f, 0, q};
        break;
    }
    return result;
}

static const GLfloat vertexData[] = {
    -BOID_SIZE / 2,
    -BOID_SIZE,
    0.0f,
    BOID_SIZE / 2,
    -BOID_SIZE,
    0.0f,
    0.0f,
    BOID_SIZE,
    0.0f,
};

struct RadVelocity
{
    float speed, angle;
};

vec2 velocityFromRadVelocity(RadVelocity radvel)
{
    return vec2(radvel.speed * cos(radvel.angle), radvel.speed * sin(radvel.angle));
}

RadVelocity radVelocityFromVelocity(vec2 velocity)
{
    return {length(velocity), atan2(velocity.y, velocity.x)};
}

// Boid object with position vector and radvelocity vector
struct Boid
{
    // Position vector
    vec2 position;
    // RadVelocity vector
    RadVelocity rvelocity;
    // vec3 color
    vec3 color;
};

class BoidGenerator
{
public:
    BoidGenerator()
        : gen(rd()),
          posdist(-1.0f, 1.0f),
          angledist(0.0f, TWO_PI),
          colordist(0.0f, 1.0f)
    {
    }
    Boid makeBoid()
    {
        return Boid{vec2(posdist(gen), posdist(gen)),
                    {0.01f, angledist(gen)},
                    vec3(1.0f, 1.0f, 1.0f)};
    }
    Boid makeBoid(float hue)
    {
        return Boid{vec2(posdist(gen), posdist(gen)),
                    {0.01f, angledist(gen)},
                    hue2rgb(hue)};
    }
    vector<Boid> init_boids(int num, bool color = true)
    {
        vector<Boid> output;
        if (num <= 0)
            return output;

        float currhue = 0.0f;
        float deltahue = 360.0f / num;
        for (int i = 0; i < num; ++i)
        {
            if (color)
                output.push_back(makeBoid(currhue));
            else
                output.push_back(makeBoid());
            currhue += deltahue;
        }
        return output;
    };

private:
    random_device rd;
    mt19937 gen;
    uniform_real_distribution<float> posdist;
    uniform_real_distribution<float> angledist;
    uniform_real_distribution<float> colordist;
};

class BoidRunner
{
public:
    BoidRunner(vector<Boid> boids) : flock(boids) {};
    vector<Boid> getFlock() { return flock; }
    void updateFlock()
    {
        for (Boid &currBoid : flock)
        {
            updateBoid(currBoid);

            // loop back on window edges
            if (currBoid.position.x > 1.0f || currBoid.position.x < -1.0f)
            {
                currBoid.position.x *= -1.0f;
                // currBoid.rvelocity.angle = PI - currBoid.rvelocity.angle;
            }
            if (currBoid.position.y > 1.0f || currBoid.position.y < -1.0f)
            {
                currBoid.position.y *= -1.0f;
                // currBoid.rvelocity.angle = TWO_PI - currBoid.rvelocity.angle;
            }
        }
    };

private:
    // NOTE: Nothing about the flock involves order so a vector may not be ideal
    vector<Boid> flock;
    /* vector<Boid> getWithinRadius(Boid theBoid, float radius)
    {
        vector<Boid> withinRadius;
        for (Boid &currBoid : flock)
        {
            if (&currBoid == &theBoid)
                continue;
            if (length(currBoid.position - theBoid.position) <= radius)
                withinRadius.push_back(currBoid);
        }
        return withinRadius;
    }
    vec2 getGroupAvgPos(vector<Boid> group)
    {
        vec2 avgPos(0, 0);
        for (Boid &currBoid : group)
        {
            avgPos.x += currBoid.position.x;
            avgPos.y += currBoid.position.y;
        }
        avgPos.x /= group.size();
        avgPos.y /= group.size();
        return avgPos;
    }
    float getGroupAvgHeading(vector<Boid> group)
    {
        float avgHeading = 0.0f;
        for (Boid &currBoid : group)
        {
            avgHeading += currBoid.rvelocity.angle;
        }
        avgHeading /= group.size();
        return avgHeading;
    }
    vec2 getSeparated(Boid theBoid, vector<Boid> group)
    {
        vec2 closevec(0, 0);
        for (Boid &currBoid : group)
        {
            closevec += theBoid.position - currBoid.position;
        }
        return closevec;
    } */
    
    void updateBoid(Boid &theBoid)
    {
        vec2 pos = theBoid.position;

        int numNear = 0;
        float avgHeading = 0;
        vec2 avgPos(0, 0);
        vec2 closeVec(0, 0);

        for (Boid &other : flock)
        {
            if (&other != &theBoid && length(other.position - pos) <= BOID_VIEW)
            {
                numNear++;
                avgPos += other.position;
                avgHeading += other.rvelocity.angle;

                if (length(other.position - pos) <= BOID_CLOSE)
                {
                    closeVec += pos - other.position;
                }
            }
        }
        if(numNear > 0)
        {
            avgPos.x /= numNear;
            avgPos.y /= numNear;
            avgHeading /= numNear;
        } else {
            avgPos = pos;
            avgHeading = theBoid.rvelocity.angle;
        }

        // Separation
        theBoid.rvelocity.angle += SEPARATION * (atan2(closeVec.y, closeVec.x) - theBoid.rvelocity.angle);

        // Alignment
        theBoid.rvelocity.angle += ALIGNMENT * (avgHeading - theBoid.rvelocity.angle);

        // Cohesion
        vec2 diff = avgPos - pos;
        float diffAngle = atan2(diff.y, diff.x);
        theBoid.rvelocity.angle += COHESION * (diffAngle - theBoid.rvelocity.angle);

        // Update position
        theBoid.position += velocityFromRadVelocity(theBoid.rvelocity);
    }
};

static void printShaderError(GLuint shader)
{
    GLint infoLogLength = 0;
    glGetShaderiv(shader, GL_INFO_LOG_LENGTH, &infoLogLength);
    if (infoLogLength > 0)
    {
        char *infoLog = static_cast<char *>(std::malloc(infoLogLength));
        glGetShaderInfoLog(shader, infoLogLength, nullptr, infoLog);
        fprintf(stderr, "%s\n", infoLog);
        std::free(infoLog);
    }
}

static void printProgramError(GLuint program)
{
    GLint infoLogLength = 0;
    glGetProgramiv(program, GL_INFO_LOG_LENGTH, &infoLogLength);
    if (infoLogLength > 0)
    {
        char *infoLog = static_cast<char *>(std::malloc(infoLogLength));
        glGetProgramInfoLog(program, infoLogLength, nullptr, infoLog);
        fprintf(stderr, "%s\n", infoLog);
        std::free(infoLog);
    }
}

static GLuint compileShader(GLenum type, const char *source)
{
    GLuint shader = glCreateShader(type);
    glShaderSource(shader, 1, &source, nullptr);
    glCompileShader(shader);

    GLint success = 0;
    glGetShaderiv(shader, GL_COMPILE_STATUS, &success);
    if (!success)
    {
        fprintf(stderr, "Shader compilation failed.\n");
        printShaderError(shader);
        glDeleteShader(shader);
        return 0;
    }

    return shader;
}

int main()
{
    glewExperimental = true;
    if (!glfwInit())
    {
        fprintf(stderr, "Failed to initialize GLFW.\n");
        return -1;
    }

    glfwWindowHint(GLFW_SAMPLES, 4);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
    glfwWindowHint(GLFW_RESIZABLE, GLFW_FALSE);

    GLFWmonitor* monitor = glfwGetPrimaryMonitor();
    const GLFWvidmode* mode = glfwGetVideoMode(monitor);

    GLFWwindow *window = glfwCreateWindow(mode->width, mode->height, "Boids", monitor, NULL);
    if (window == nullptr)
    {
        fprintf(stderr, "Failed to open GLFW window.\n");
        glfwTerminate();
        return -2;
    }

    glfwMakeContextCurrent(window);
    if (glewInit() != GLEW_OK)
    {
        fprintf(stderr, "Failed to initialize GLEW.\n");
        glfwDestroyWindow(window);
        glfwTerminate();
        return -3;
    }

    glfwSetInputMode(window, GLFW_STICKY_KEYS, GL_TRUE);

    GLuint vertexArrayId = 0;
    glGenVertexArrays(1, &vertexArrayId);
    glBindVertexArray(vertexArrayId);

    GLuint vertexBuffer = 0;
    glGenBuffers(1, &vertexBuffer);
    glBindBuffer(GL_ARRAY_BUFFER, vertexBuffer);
    glBufferData(GL_ARRAY_BUFFER, sizeof(vertexData), vertexData, GL_STATIC_DRAW);

    const char *vertexShaderSource = R"(
        #version 330 core
        layout(location = 0) in vec3 aPos;
        uniform mat4 uModel;

        void main()
        {
            gl_Position = uModel * vec4(aPos, 1.0);
        }
    )";

    const char *fragmentShaderSource = R"(
        #version 330 core
        out vec4 FragColor;
        uniform vec3 uColor;
        void main()
        {
            FragColor = vec4(uColor, 1.0);
        }
    )";

    GLuint vertexShader = compileShader(GL_VERTEX_SHADER, vertexShaderSource);
    if (vertexShader == 0)
    {
        glfwDestroyWindow(window);
        glfwTerminate();
        return -4;
    }

    GLuint fragmentShader = compileShader(GL_FRAGMENT_SHADER, fragmentShaderSource);
    if (fragmentShader == 0)
    {
        glDeleteShader(vertexShader);
        glfwDestroyWindow(window);
        glfwTerminate();
        return -5;
    }

    GLuint program = glCreateProgram();
    glAttachShader(program, vertexShader);
    glAttachShader(program, fragmentShader);
    glLinkProgram(program);

    GLint linkStatus = 0;
    glGetProgramiv(program, GL_LINK_STATUS, &linkStatus);
    if (!linkStatus)
    {
        fprintf(stderr, "Program linking failed.\n");
        printProgramError(program);
        glDeleteProgram(program);
        glDeleteShader(vertexShader);
        glDeleteShader(fragmentShader);
        glfwDestroyWindow(window);
        glfwTerminate();
        return -6;
    }

    glUseProgram(program);
    glEnableVertexAttribArray(0);
    glBindBuffer(GL_ARRAY_BUFFER, vertexBuffer);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 0, nullptr);

    GLint modelLoc = glGetUniformLocation(program, "uModel");
    glClearColor(0.0f, 0.0f, 0.0f, 1.0f);

    BoidGenerator bgen;
    BoidRunner theBoids(bgen.init_boids(NUM_BOIDS, true));

    while (glfwGetKey(window, GLFW_KEY_ESCAPE) != GLFW_PRESS &&
           glfwWindowShouldClose(window) == 0)
    {

        // 1. Update boids
        theBoids.updateFlock();

        // 2. Render
        glClear(GL_COLOR_BUFFER_BIT);
        glUseProgram(program);

        for (Boid &currBoid : theBoids.getFlock())
        {
            glUniform3fv(glGetUniformLocation(program, "uColor"), 1, &currBoid.color[0]);
            mat4 model = translate(mat4(1.0f), vec3(currBoid.position, 0.0f));
            model = rotate(model, currBoid.rvelocity.angle - HALF_PI, vec3(0.0f, 0.0f, 1.0f));
            glUniformMatrix4fv(modelLoc, 1, GL_FALSE, &model[0][0]);
            glDrawArrays(GL_TRIANGLES, 0, 3);
        }

        glfwSwapBuffers(window);
        glfwPollEvents();
    }

    glDeleteProgram(program);
    glDeleteShader(vertexShader);
    glDeleteShader(fragmentShader);
    glDeleteBuffers(1, &vertexBuffer);
    glDeleteVertexArrays(1, &vertexArrayId);

    glfwDestroyWindow(window);
    glfwTerminate();
    return 0;
}