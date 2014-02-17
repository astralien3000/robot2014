#include <termios.h>
#include <fcntl.h>
#include <unistd.h>
#include <string.h>

#include <iostream>
#include <sstream>
#include <fstream>
using namespace std;


#include <QtWidgets/QMainWindow>
#include <QtWidgets/QApplication>
#include <QtWidgets/QTreeWidgetItem>

#include <QtCore/QIODevice>
#include <QtCore/QTimer>
#include <QtCore/QThread>
#include <QtCore/QMutex>
#include "ui_main_window.hpp"

QMutex lock, filelock;
/*/

stringstream oss;

class Test : public QThread {
  Q_OBJECT

private:
  int _fd;

public:
  Test(int fd) : _fd(fd) {}

private slots:
  void run(void) {
    fstream of("test", ios_base::out);
    usleep(1000000);
    while(1) {
      char buff[1024];
      ::read(_fd, buff, 1024);
      lock.lock();
      oss.write(buff, strlen(buff));
      lock.unlock();
      //oss.flush();
      of << buff;
      of.flush();
      //sleep(0);
    }
  }
};

class SerialDevice : public QIODevice {
  Q_OBJECT

private:
  int _fd;

public:
  SerialDevice(void) {
    struct termios serial_cfg;

    // Opening file
    if((_fd = ::open("/dev/ttyACM0", O_RDWR | O_NOCTTY)) == -1) {
      cerr << "ERROR : cannot open device..." << endl;
    }

    fcntl(0, F_SETFL, O_NONBLOCK);

    // Checkin if file is a tty
    if(!isatty(_fd)) {
      cerr << "ERROR : not a serial file" << endl;
    }

    // Getting tty configuration
    if(tcgetattr(_fd, &serial_cfg) < 0) {
      cerr << "ERROR : unable to get configuration" << endl;
    }

    // Setting input flags
    //serial_cfg.c_iflag &= ~(IXON | IXOFF | IXANY);
    //serial_cfg.c_iflag &= ~(ICANON | ECHO | ECHOE | ISIG);
    

    // Setting output flags
    //serial_cfg.c_oflag &= ~OPOST;

    // Setting control flags
    //serial_cfg.c_cflag &= ~(PARENB | CSTOPB | CSIZE | CRTSCTS);
    //serial_cfg.c_cflag |= CS8 | CREAD | CLOCAL;

    // Setting local mode flags
    //serial_cfg.c_lflag = 0;
  
    // Setting special characters
    serial_cfg.c_cc[VMIN]  = 12;
    serial_cfg.c_cc[VTIME] = 5;
  
    // Setting baudrate
    if(cfsetispeed(&serial_cfg, B9600) < 0) {
      cerr << "ERROR : in Baudrate configuration failed" << endl;
    }
    if(cfsetospeed(&serial_cfg, B9600) < 0) {
      cerr << "ERROR : out Baudrate configuration failed" << endl;
    }

    // Writing new configuration
    if(tcsetattr(_fd, TCSAFLUSH, &serial_cfg) < 0) {
      cerr << "ERROR : cannot configure tty" << endl;
    }

    setOpenMode(QIODevice::ReadWrite);
    
    Test* t = new Test(_fd);
    t->start();
  }

protected:
  qint64 readData(char* data, qint64 maxSize) {
    data[0] = 0;
    lock.lock();
    oss.getline(data, maxSize);
    lock.unlock();
    return ((long long)oss.gcount());
  }

  qint64 writeData(const char* data, qint64 maxSize) {
    qint64 ret = ::write(_fd, data, maxSize);
    return ret;
  }
};

class MyTest : public QObject {
  Q_OBJECT

  QTextEdit* _te;
  SerialDevice* _sd;
  
public:
  MyTest(QTextEdit* te, SerialDevice* sd) : _te(te), _sd(sd) {}
    
public slots:
  void test(void) {
    char buff[1024] = {0};
    _sd->readLine(buff, 1024);

    _te->insertPlainText(buff);

    _sd->write("100\n");
  }
};

*/
Ui_MainWindow ui;

class Test : public QThread {
  Q_OBJECT

  public:
  Test() {}

private slots:
  void run(void) {
    usleep(1000000);

    
    int fd1 = open("/dev/ttyACM0", O_RDONLY);
    dup2(fd1, 0);

    while(1) {
      char buff[1024];

      filelock.lock();
      cin.getline(buff, 1024);
      filelock.unlock();
      QStringList l = QString(buff).split(" ");

      if(l.at(0) == "test") {
      	emit send(l.at(2));
      }
      usleep(0);
    }
  }

signals:
  void send(QString);
};

class Test2 : public QThread {
  Q_OBJECT

  bool test;

  public:
  Test2() : test(false) {}

private slots:
  void run(void) {
    usleep(1000000);

    // int fd1 = open("/dev/ttyACM0", O_WRONLY | O_NONBLOCK);
    // if(fd1 < 0) {
    //   cerr << "ERROR"<< endl;
    // }
    // dup2(fd1, 1);
    volatile int i = 0, count = 0;

    while(1) {
      lock.lock();
      if(test) {
	if(filelock.tryLock(10)) {
	  cout << i << endl;
	  filelock.unlock();
	}
	
	i++;
	count++;
      }
      test = false;
      lock.unlock();
      usleep(10);
    }
  }

public slots:
  void send() {
    if(lock.tryLock(10)) {
      test = true;
      lock.unlock();
    }
  }
};


class EvtRecv : public QObject{
  Q_OBJECT
public:
  EvtRecv() {}

public slots:
  void test(QString str) {
    ui.system_view->itemAt(0,0)->setText(1, str);
    ui.system_view->itemAt(0,0)->setText(0, QString("miew!"));
  }
};


#include "moc_main.cpp"


//*/
int main(int argc, char* argv[]) {
  QApplication app(argc, argv);

  QMainWindow win;
  ui.setupUi(&win);

  int fd2 = open("/dev/ttyACM0", O_WRONLY | O_NOCTTY);
  dup2(fd2, 1);

  //int fd1 = open("/dev/ttyACM0", O_RDONLY);
  //dup2(fd1, 0);

  Test t;
  t.start();
  
  Test2 t2;
  t2.start();

  EvtRecv r;
  app.connect(&t, SIGNAL(send(QString)), &r, SLOT(test(QString)));

  win.show();

   char buff[1024];
  while(1) {
    t2.send();
    app.processEvents();
    usleep(10);
  }

  return app.exec();
}

/*/
int main(int argc, char* argv[]) {
  int serial_fd;
  struct termios serial_cfg;
  char buff[1024] = {0};

  // Opening file
  if((serial_fd = open("/dev/ttyACM0", O_RDWR | O_NONBLOCK)) == -1) {
    cerr << "ERROR : cannot open device..." << endl;
  }

  fcntl(0, F_SETFL, O_NONBLOCK);

  // Checkin if file is a tty
  if(!isatty(serial_fd)) {
    cerr << "ERROR : not a serial file" << endl;
  }

  // Getting tty configuration
  if(tcgetattr(serial_fd, &serial_cfg) < 0) {
    cerr << "ERROR : unable to get configuration" << endl;
  }

  // Setting input flags
  serial_cfg.c_iflag = 0;

  // Setting output flags
  serial_cfg.c_oflag = 0;

  // Setting control flags
  serial_cfg.c_cflag = CS8;

  // Setting local mode flags
  serial_cfg.c_lflag = 0;
  
  // Setting special characters
  serial_cfg.c_cc[VMIN]  = 1;
  serial_cfg.c_cc[VTIME] = 0;
  
  // Setting baudrate
  if(cfsetispeed(&serial_cfg, B9600) < 0) {
    cerr << "ERROR : in Baudrate configuration failed" << endl;
  }
  if(cfsetospeed(&serial_cfg, B9600) < 0) {
    cerr << "ERROR : out Baudrate configuration failed" << endl;
  }

  // Writing new configuration
  if(tcsetattr(serial_fd, TCSAFLUSH, &serial_cfg) < 0) {
    cerr << "ERROR : cannot configure tty" << endl;
  }

  usleep(1000000);
  cout << "Connected" << endl;

  bool keep = true;
  char out_buff[1024];
  while(keep) {
    for(int i = 0 ; i < sizeof(buff) ; i++) {
      buff[i] = 0;
      out_buff[i] = 0;
    }

    if(0 < read(serial_fd, out_buff, sizeof(buff))) {
      write(1, out_buff, strlen(out_buff));
    }

    if(0 < read(0, buff, sizeof(buff))) {
      write(serial_fd, buff, strlen(buff));
    }
  }

  close(serial_fd);
  return 0;
}
//*/
