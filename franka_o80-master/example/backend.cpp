#include "../include/franka_o80/driver.hpp"
#include "../include/franka_o80/standalone.hpp"
#include "../include/franka_o80/version.hpp"
#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <stdlib.h>
#include <signal.h>
#include <iostream>
#include <thread>
#include <chrono>

namespace franka_o80
{
    void help();
    int start(int argc, char **argv);
    int stop(int argc, char **argv);
    int status(int argc, char **argv);
    int run(int argc, char **argv);
}

void franka_o80::help()
{
    std::cout << "Welcome to franka_o80_backend " << franka_o80::version_major << "." << franka_o80::version_minor << "." << franka_o80::version_patch << "!" << std::endl;
    std::cout << "The program is created to start and stop backends"          << std::endl;
    std::cout << std::endl;
    std::cout << "Usage:"                                                     << std::endl;
    std::cout << "./backend start  ID IP      to start backend"               << std::endl;
    std::cout << "./backend status ID         to check if backend is running" << std::endl;
    std::cout << "./backend stop   ID         to stop backend"                << std::endl;
    std::cout << "./backend        ID IP      to run backend in shell"        << std::endl;
}

int franka_o80::start(int argc, char **argv)
{
    //Reading ID
    const char *id = argv[2];

    //Reading IP
    const char *ip = argv[3];

    //Creating daemon
    pid_t pid = fork();
    if (pid < 0) return 1;
    if (pid == 0)
    {
        setsid();
        signal(SIGCHLD, SIG_IGN);
        signal(SIGHUP, SIG_IGN);
        umask(0);
        chdir("/");
        std::cout << "Starting backend" << std::endl;
        for (int x = sysconf(_SC_OPEN_MAX); x >= 0; x--) close (x);
        franka_o80::start_standalone(id, ip);
    }
    return 0;
}

int franka_o80::stop(int argc, char **argv)
{
    //Reading ID
    const char *id = argv[1];

    //Stopping standalone
    std::cout << "Stopping backend" << std::endl;
    franka_o80::stop_standalone(id);
    return 0;
}

int franka_o80::status(int argc, char **argv)
{
    //Reading ID
    const char *id = argv[1];
    
    //Reading standalone status
    if (franka_o80::standalone_is_running(id)) std::cout << "1" <<std::endl;
    else std::cout << "0" <<std::endl;

    return 0;
}

int franka_o80::run(int argc, char **argv)
{
    //Reading ID
    static const char *id = argv[1];

    //Reading IP
    static const char *ip = argv[2];
    
    //Starting standalone
    std::cout << "Starting backend" << std::endl;
    franka_o80::start_standalone(id, ip);
    static bool finish = false;
    
    //Main loop
    signal(SIGINT, [](int sig) { if (sig == SIGINT) finish = true; });
    while (!finish) std::this_thread::sleep_for(std::chrono::milliseconds(100));

    //Stopping standalone
    std::cout << "Stopping backend" << std::endl;
    franka_o80::please_stop(id);
    while (franka_o80::standalone_is_running(id)) std::this_thread::sleep_for(std::chrono::milliseconds(100));
            
    return 0;
}

int main(int argc, char **argv)
{
    try
    {
        if (argc == 4 && std::string(argv[1]) == "start") return franka_o80::start(argc, argv);
        else if (argc == 3 && std::string(argv[1]) == "status") return franka_o80::status(argc, argv);
        else if (argc == 3 && std::string(argv[1]) == "stop") return franka_o80::stop(argc, argv);
        else if (argc == 3) return franka_o80::run(argc, argv);
        else franka_o80::help();
    }
    catch (std::exception &e)
    {
        std::cout << "Exception occured: " << e.what() << std::endl;
    }
    return 1;
}