#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <signal.h>
#include <sys/wait.h>
#include <arpa/inet.h>
#include <sys/socket.h>
void read_childproc(int sig)
{
    int status;
    pid_t id = waitpid(-1, &status, WNOHANG);
    if (WIFEXITED(status))
    {
        printf("Removed proc id: %d \n", id);
        printf("Child send: %d \n", WEXITSTATUS(status));
    }
}
void error_handling(char *message)
{
    fputs(message, stderr);
    fputc('\n', stderr);
    exit(1);
}
int main() {
    struct sigaction act;
    act.sa_handler = read_childproc;
    sigemptyset(&act.sa_mask);
    act.sa_flags = 0;
    sigaction(SIGCHLD, &act, 0);
    // 1.创建一个通信的socket, 注意第二个参数是：SOCK_DGRAM，数据报的协议。
    int fd = socket(PF_INET, SOCK_DGRAM, 0);
    if(fd == -1)   
        error_handling("socket create error");
    struct sockaddr_in addr;
    addr.sin_family = AF_INET;
    addr.sin_port = htons(9999);
    addr.sin_addr.s_addr = INADDR_ANY;	//--服务器绑定本机所有的网卡
    // 2.绑定
    int ret = bind(fd, (struct sockaddr *)&addr, sizeof(addr));
    if(ret == -1) 
        error_handling("bind error");
    // 3.通信
    char recvbuf[512];
    struct sockaddr_in cliaddr;
    socklen_t len = sizeof(cliaddr);	//--用来保存客户端的地址
    // 接收数据
    pid_t vins_pid = 0, px4ctrl_pid = 0, takeoff_pid = 0, ego_planner_pid = 0;
    while (1) {
        memset(recvbuf, 0, sizeof(recvbuf));
        int num = recvfrom(fd, recvbuf, sizeof(recvbuf), 0, (struct sockaddr *)&cliaddr, &len);
        printf("cmd received: %s\n", recvbuf);
        if (strcmp(recvbuf, "vins") == 0 && vins_pid == 0)
        {
            switch (vins_pid = fork()) {
            case -1:
                error_handling("fork error");
            case 0:
                FILE *pip_output;
                if ((pip_output = popen("roslaunch px4 indoor1.launch 2>&1", "r")) == NULL)
                    error_handling("popen error");
                while (1) {
                    if (fgets(recvbuf, 512, pip_output))
                        sendto(fd, recvbuf, strlen(recvbuf) + 1, 0, (struct sockaddr *)&cliaddr, sizeof(cliaddr));
                }
                exit(0);
            default:
                printf("创建了子进程%d\n", vins_pid);
                break;
            }
        }
        else if (strcmp(recvbuf, "px4ctrl") == 0) {
            for (int i = 0; i < 10; i++) {
                char buf[20];
                sprintf(buf, "px4ctrl message %d", i+1);
                sendto(fd, buf, strlen(buf) + 1, 0, (struct sockaddr *)&cliaddr, sizeof(cliaddr));
            }
        } else if (strcmp(recvbuf, "takeoff") == 0) {
            for (int i = 0; i < 10; i++) {
                char buf[20];
                sprintf(buf, "takeoff message %d", i+1);
                sendto(fd, buf, strlen(buf) + 1, 0, (struct sockaddr *)&cliaddr, sizeof(cliaddr));
            }
        } else if (strcmp(recvbuf, "planner") == 0) {
            for (int i = 0; i < 10; i++) {
                char buf[20];
                sprintf(buf, "planner message %d", i+1);
                sendto(fd, buf, strlen(buf) + 1, 0, (struct sockaddr *)&cliaddr, sizeof(cliaddr));
            }
        }
    }
    close(fd);
    return 0;
}