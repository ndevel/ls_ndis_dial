#include <stdio.h> /*标准输入输出定义*/
#include <errno.h> /*错误号定义*/
#include <sys/stat.h>
#include <fcntl.h>   /*文件控制定义*/
#include <termios.h> /*POSIX 终端控制定义*/
#include <stdlib.h>  /*标准函数库定义*/
#include <sys/types.h>
#include <unistd.h> /*UNIX 标准函数定义*/
#include <string.h>

#define serial_device "/dev/ttyUSB1"

/***********************************************************
* @函数：open_port
* @功能：打开串口
* @参数：
* @作者：ghui
************************************************************/
int open_port(void)
{
    int fd = -1;                                            //串口标识符
                                                            /*
	O_RDONLY 	只读打开。
	O_WRONLY 	只写打开。
	O_RDWR 		读、写打开。
	O_APPEND 	每次写时都加到文件的尾端。
	O_CREAT 	若此文件不存在则创建它。使用此选择项时，需同时说明第三个参数mode，用其说明该新文件的存取许可权位。
	O_EXCL 		如果同时指定了O_CREAT，而文件已经存在，则出错。这可测试一个文件是否存在，如果不存在则创建此文件成为一个原子操作。
	O_TRUNC 	如果此文件存在，而且为只读或只写成功打开，则将其长度截短为0。
	O_NOCTTY 	如果 pathname 指的是终端设备，则不将此设备分配作为此进程的控制终端。
	O_NDELAY	通知linux系统不关心DCD信号线所处的状态(端口的另一端是否**或者停止)。	
	O_NONBLOCK 	如果 pathname 指的是一个FIFO、一个块特殊文件或一个字符特殊文件，则此选择项为此文件的本次打开操作和后续的 I/O 操作设置非阻塞方式。
	O_SYNC 		使每次 write 都等到物理 I/O 操作完成	
*/
    fd = open(serial_device, O_RDWR | O_NOCTTY | O_NDELAY); //以读写方式、不把该文件作为终端设备、无延时模式打开串口
    if (fd == -1)
    {
        //不能打开串口
        perror("Open_port: Unable to open /dev/ttyUSB1 \n");
        return (fd);
    }
    else
    {
        fcntl(fd, F_SETFL, 0); //设置文件flag为0（默认），即阻塞状态
        printf("Opened /dev/ttyUSB1 \n");
        return (fd);
    }
}

#if 0
/***********************************************************
* @函数：serial_write
* @功能：串口数据发送函数
* @参数：	fd：串口描述符
			data：待发送数据
			datalen：数据长度
* @作者：ghui
************************************************************/
int serial_write(int fd ,char *data, int datalen)
{
	int len=0;
 
	len = write(fd,data,datalen);
	printf("send data OK! datalen=%d\n",len);
	return len;	
}
 
/***********************************************************
* @函数：serial_read
* @功能：串口数据接收函数
* @参数：	fd：串口描述符
			buff：接收的数据
			datalen：数据长度
* @作者：ghui
************************************************************/
int serial_read(int fd,char buff[],int datalen)
{
	int nread=0;
	printf("Ready for receiving data...");
	nread = read(fd,buff,datalen);
	if(nread>0)
	{
		printf("Revlength=%d \n",nread);
		buff[nread]='\0';
		printf("%s\n",buff);
	}
	return nread;
}
#endif

/***********************************************************
* @函数：set_serial
* @功能：串口属性配置
* @参数：	nSpeed：波特率
			nBits：	数据位
			nEvent：奇偶校验位
			nStop：	停止位
* @作者：ghui
************************************************************/
int set_serial(int fd, int nSpeed, int nBits, char nEvent, int nStop)
{
    struct termios newtty, oldttys;

    /*保存原有串口配置*/
    if (tcgetattr(fd, &oldttys) != 0)
    {
        perror("Setupserial 1");
        return -1;
    }
    bzero(&newtty, sizeof(newtty));
    newtty.c_cflag |= (CLOCAL | CREAD);
    /*
		CREAD：开启串行数据接收，保证程序可以从串口中读取数据
        CLOCAL：打开本地连接模式，保证程序不占用串口
	*/

    newtty.c_cflag &= ~CSIZE; //字符长度，设置数据位之前一定要屏掉这个位
    /*数据位选择*/
    switch (nBits)
    {
    case 7:
        newtty.c_cflag |= CS7;
        break;
    case 8:
        newtty.c_cflag |= CS8;
        break;
    default:
        break;
    }
    /*设置奇偶校验位*/
    switch (nEvent)
    {
    case '0':                               /*奇校验*/
        newtty.c_cflag |= PARENB;           /*开启奇偶校验*/
        newtty.c_iflag |= (INPCK | ISTRIP); /*INPCK打开输入奇偶校验；ISTRIP去除字符的第八个比特  */
        newtty.c_cflag |= PARODD;           /*启用奇校验(默认为偶校验)*/
        break;
    case 'E':                               /*偶校验*/
        newtty.c_cflag |= PARENB;           /*开启奇偶校验  */
        newtty.c_iflag |= (INPCK | ISTRIP); /*打开输入奇偶校验并去除字符第八个比特*/
        newtty.c_cflag &= ~PARODD;          /*启用偶校验*/
        break;
    case 'N': /*无奇偶校验*/
        newtty.c_cflag &= ~PARENB;
        break;
    default:
        break;
    }
    /*设置波特率*/
    switch (nSpeed)
    {
    case 2400:
        cfsetispeed(&newtty, B2400);
        cfsetospeed(&newtty, B2400);
        break;
    case 4800:
        cfsetispeed(&newtty, B4800);
        cfsetospeed(&newtty, B4800);
        break;
    case 9600:
        cfsetispeed(&newtty, B9600);
        cfsetospeed(&newtty, B9600);
        break;
    case 115200:
        cfsetispeed(&newtty, B115200);
        cfsetospeed(&newtty, B115200);
        break;
    default:
        cfsetispeed(&newtty, B9600);
        cfsetospeed(&newtty, B9600);
        break;
    }
    /*设置停止位*/
    if (nStop == 1)
    {                              /*设置停止位；若停止位为1，则清除CSTOPB，若停止位为2，则**CSTOPB*/
        newtty.c_cflag &= ~CSTOPB; /*默认为一位停止位； */
    }
    else if (nStop == 2)
    {
        newtty.c_cflag |= CSTOPB; /*CSTOPB表示送两位停止位*/
    }

    /*--------------------其他配置-----------------------*/
    /*设置本地模式为原始模式*/
    newtty.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
    /*
		ICANON：允许规范模式进行输入处理
		ECHO：允许输入字符的本地回显
		ECHOE：在接收EPASE时执行Backspace,Space,Backspace组合
		ISIG：允许信号
	*/

    /*设置输出模式为原始输出*/
    newtty.c_oflag &= ~OPOST; //OPOST：若设置则按定义的输出处理，否则所有c_oflag失效
                              /*发送字符0X0d的时候，往往接收端得到的字符是0X0a，原因是因为在串口设置中c_iflag和c_oflag中存在从NL-CR和CR-NL的映射，即串口能把回车和换行当成同一个字符，可以进行如下设置屏蔽之*/
    newtty.c_oflag &= ~(ONLCR | OCRNL);
    newtty.c_iflag &= ~(ICRNL | INLCR);
    newtty.c_cflag &= ~CRTSCTS;                                  // 不使用硬件数据流控制
    newtty.c_iflag &= ~(BRKINT | ICRNL | INPCK | ISTRIP | IXON); /*c_cc数组的VSTART和VSTOP元素被设定成DC1和DC3，代表ASCII标准的XON和XOFF字符，如果在传输这两个字符的时候就传不过去，需要把软件流控制屏蔽*/

    /*设置最小字符数和等待时间，对于接收字符和等待时间没有特别的要求时*/
    newtty.c_cc[VTIME] = 0; /*非规范模式读取时的超时时间；单位：百毫秒*/
    newtty.c_cc[VMIN] = 0;  /*非规范模式读取时的最小字符数*/
    tcflush(fd, TCIFLUSH);  /*tcflush清空终端未完成的输入/输出请求及数据*/
                            /*
		TCIFLUSH：	清除正收到的数据，且不会读取出来。
		TCOFLUSH：	清除正写入的数据，且不会发送至终端。
		TCIOFLUSH：	清除所有正在发生的I/O数据。
		TCOOFF：	挂起输出
		TCOON：		重启挂起的输出
		TCIOFF：	发送一个停止位
		TCION：		发送一个起始位
	*/

    /***配置使其生效*/
    if ((tcsetattr(fd, TCSANOW, &newtty)) != 0)
    {
        perror("com set error");
        return -1;
    }
    /*
		TCSANOW:不等数据传输完毕就立即改变属性。
		TCSADRAIN:等待所有数据传输结束才改变属性。
		TCSAFLUSH:等待所有数据传输结束,清空输入输出缓冲区才改变属性。
	*/
    return 0;
}

/*
main
*/
int main(void)
{
    int fd;
    const char message_disconnect[] = "AT$QCRMCALL=0,1\r\n";
    const char message_connect[] = "AT$QCRMCALL=1,1\r\n";
    const char target_message[] = "$QCRMCALL: 1, V4";
    char buffer[256] = {0};

    fd = open_port();
    if (fd == -1)
    {
        return 0;
    }
    set_serial(fd, 115200, 8, 'N', 1);

    printf("Clear serial buffer.\n");
    tcflush(fd, TCIOFLUSH);

    int nCount = 0;
    while (nCount < 10)
    {
        printf("Write: %s\n", message_disconnect);
        write(fd, message_disconnect, strlen(message_disconnect));
        sleep(1);
        printf(".");
        sleep(1);
        printf(".");
        sleep(1);
        printf(".\n");

        memset(buffer, 0, strlen(buffer));
        if (read(fd, buffer, sizeof(buffer)) > 0)
        {
            printf("Read response: %s\n", buffer);
        }
        else
        {
            printf("Read nothing.\n");
        }

        printf("Write: %s\n", message_connect);
        write(fd, message_connect, strlen(message_connect));
        sleep(1);
        printf(".");
        sleep(1);
        printf(".");
        sleep(1);
        printf(".\n");

        memset(buffer, 0, strlen(buffer));
        if (read(fd, buffer, sizeof(buffer)) > 0)
        {
            printf("Read response: %s\n", buffer);

            if (strstr(buffer, target_message) != NULL)
            {
                printf("Dial OK.\n");
                break;
            }
            else
            {
                printf("Dial failed, try again later.\n");
            }
        }
        else
        {
            printf("Read nothing.\n");
        }

        ++nCount;
    }

    close(fd);

    return 0;
}
