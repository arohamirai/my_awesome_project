#include <errno.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/types.h>
#include <sys/wait.h>
#include <unistd.h>

#include <iostream>
#include <signal.h>
bool g_flag = false;
int i = 0;
void callback_func(int a, siginfo_t* si, void* c)
{
     int x = *((int*)si->si_ptr);

  std::cout <<"i: "<< i <<", a: " << a << ", x:" << x << std::endl;
  i++;
  //g_flag = true;
}

int main(int argc, char** argv)
{
  int xxx = 119;
  union sigval val;  //定义一个携带数据的共用体
  val.sival_ptr = &xxx;

  //信号处理函数注册
  struct sigaction act, oldact;
  act.sa_sigaction = callback_func;
  act.sa_flags = SA_SIGINFO;

  // sigaction(SIGINT,&act,&oldact);
  sigaction(SIGUSR1, &act, &oldact);


  //sigqueue(getpid(),SIGUSR1,val);//向本进程发送一个信号

  while(!g_flag)
  {
      sleep(2);
      //一般而言，sigqueue与sigaction配合使用，而kill与signal配合使用。
      sigqueue(getpid(),SIGUSR1,val);//向本进程发送一个信号
  }

}
