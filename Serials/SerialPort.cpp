#include "SerialPort.hpp"


/**
 * @brief list all the available uart resources
 * @return uart's name that user appointed
 */
string get_uart_dev_name() {
    FILE *ls = popen("ls /dev/ttyTHS* --color=never", "r");
    char name[20] = {0};
    fscanf(ls, "%s", name);
    pclose(ls);
    return name;
}

/**
 * @brief constructor of Serial class
 * @param nSpeed baud rate
 * @param nEvent verification mode, odd parity, even parity or none
 * @param nBits the length of data
 * @param nStop length of stop bits
 * @return none
 */
Serial::Serial(int nSpeed, char nEvent, int nBits, int nStop) :
        nSpeed(nSpeed), nEvent(nEvent), nBits(nBits), nStop(nStop) {
    serial_state = InitPort(nSpeed, nEvent, nBits, nStop);
}

/**
 * @brief destructor of Serial class
 * @return none
 */
Serial::~Serial() {
    close(fd);
    fd = -1;
}

/**
 * @brief initialize port
 * @param nSpeed_ baud rate
 * @param nEvent_ verification mode, odd parity, even parity or none
 * @param nBits_ the length of data
 * @param nStop_ length of stop bits
 * @return none
 */
bool Serial::InitPort(int nSpeed_, char nEvent_, int nBits_, int nStop_) {
    string name = PC2STM32;
    if (name.empty()) {
        return false;
    }
    if ((fd = open(name.data(), O_RDWR | O_APPEND | O_SYNC)) < 0) {
        return false;
    }
    return set_opt(fd, nSpeed_, nEvent_, nBits_, nStop_) >= 0;
}

/**
 * @brief package the data needed by lower computer
 * @param yaw yaw angle
 * @param pitch pitch angle
 * @param find found target armor or not
 * @param CarID the ID number of the car
 * @param shoot shoot or not
 * @return none
 */
void Serial::pack(float yaw, float pitch, uint8_t find, uint8_t CarID, uint8_t shoot) {
    int BLANK = 0xffffff;
    buff[0] = VISION_SOF;
    memcpy(buff + 1, &yaw, 4);
    memcpy(buff + 5, &pitch, 4);
    memcpy(buff + 9, &find, 1);
    memcpy(buff + 10, &CarID, 1);
    memcpy(buff + 11, &shoot, 1);
    memcpy(buff + 12, &BLANK, 3);
    buff[SEND_LENGTH - 1] = static_cast<char>(VISION_TOF);
}

/**
 * @brief write data to port
 * @return always should be true
 */
bool Serial::WriteData() {
    int cnt = 0, curr = 0;
    if (fd <= 0) {
        return false;
    }

    tcflush(fd, TCOFLUSH);
    curr = write(fd, buff, SEND_LENGTH);

    if (curr < 0) {
        raise(SIGINT);
        close(fd);
        return false;
    }

    return true;
}

/**
 * @brief read data sent by lower computer
 * @param buffer_ instance should be updated by data sent from lower computer
 * @return on finding the right data in a limited length of received data, return true, if not, return false
 */
bool Serial::ReadData(struct ReceiveData &buffer_) {
    memset(buffRead, 0, RECEIVE_LENGTH);
    maxReadTime = RECEIVE_LENGTH;
    static int onceReadCount = 0;

    tcflush(fd, TCIFLUSH);

    while (maxReadTime--) {
        read(fd, &buffRead[0], 1);
        if (buffRead[0] == 0xA5) break;
    }

    if (maxReadTime == 0)return false;

    readCount = 1;
    while (readCount < RECEIVE_LENGTH - 1) {
        try {
            onceReadCount = read(fd, (buffRead + readCount), RECEIVE_LENGTH - readCount);
        }
        catch (exception e) {

            return false;
        }

        if (onceReadCount < 1) {
            return false;
        }
        readCount += onceReadCount;
    }

    if (buffRead[0] != VISION_SOF || buffRead[RECEIVE_LENGTH - 1] != VISION_TOF) {
        return false;
    } else {
        memcpy(&buffer_.yawAngle, buffRead + 1, 4);
        memcpy(&buffer_.pitchAngle, buffRead + 5, 4);
        memcpy(&buffer_.bulletSpeed, buffRead + 9, 4);
        memcpy(&buffer_.targetMode, buffRead + 13, 1);
        memcpy(&buffer_.targetColor, buffRead + 14, 1);
        return true;
    }

}

/**
 * @brief set port
 * @param fd file or port descriptor
 * @param nSpeed baud rate
 * @param nEvent verification mode, odd parity, even parity or none
 * @param nBits length of data
 * @param nStop length of stop bits
 * @return
 */
int Serial::set_opt(int fd, int nSpeed, char nEvent, int nBits, int nStop) {
    termios newtio{}, oldtio{};

    if (tcgetattr(fd, &oldtio) != 0) {
        return -1;
    }
    bzero(&newtio, sizeof(newtio));
    newtio.c_cflag |= CLOCAL | CREAD;
    newtio.c_cflag &= ~CSIZE;

    switch (nBits) {
        case 7:
            newtio.c_cflag |= CS7;
            break;
        case 8:
            newtio.c_cflag |= CS8;
            break;
        default:
            break;
    }

    switch (nEvent) {
        case 'O':  //?????????
            newtio.c_cflag |= PARENB;
            newtio.c_cflag |= PARODD;
            newtio.c_iflag |= (INPCK | ISTRIP);
            break;
        case 'E':  //?????????
            newtio.c_iflag |= (INPCK | ISTRIP);
            newtio.c_cflag |= PARENB;
            newtio.c_cflag &= ~PARODD;
            break;
        case 'N':  //?????????
            newtio.c_cflag &= ~PARENB;
            break;
        default:
            break;
    }

    switch (nSpeed) {
        case 2400:
            cfsetispeed(&newtio, B2400);
            cfsetospeed(&newtio, B2400);
            break;
        case 4800:
            cfsetispeed(&newtio, B4800);
            cfsetospeed(&newtio, B4800);
            break;
        case 9600:
            cfsetispeed(&newtio, B9600);
            cfsetospeed(&newtio, B9600);
            break;
        case 115200:
            cfsetispeed(&newtio, B115200);
            cfsetospeed(&newtio, B115200);
            break;
        default:
            cfsetispeed(&newtio, B9600);
            cfsetospeed(&newtio, B9600);
            break;
    }

    if (nStop == 1) {
        newtio.c_cflag &= ~CSTOPB;
    } else if (nStop == 2) {
        newtio.c_cflag |= CSTOPB;
    }
    newtio.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
    newtio.c_oflag &= ~OPOST;
    newtio.c_cc[VTIME] = 1;
    newtio.c_cc[VMIN] = 1;
    tcflush(fd, TCIFLUSH);
    if ((tcsetattr(fd, TCSANOW, &newtio)) != 0) {
        perror("com set error");
        return -1;
    }
    printf("set done!\n");

    return 0;
}
