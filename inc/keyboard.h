#ifndef KEYBOARD_H
#define KEYBOARD_H

#include <iostream>
#include <fcntl.h>
#include <pthread.h>
#include <linux/input.h>
#include <fstream>
#include <unistd.h>
#include <fcntl.h>


#define KEYBOARD_DEV "/dev/input/event1"

struct keyboard_state {
	signed short keys[KEY_CNT];
};

class cKeyboard {
  private:
	pthread_t thread;
	int keyboard_fd;
	input_event *keyboard_ev;
	keyboard_state *keyboard_st;
	char name[256];

  protected:
  public:
	bool active;
	cKeyboard();
	~cKeyboard();
	static void* loop(void* obj);
	void readEv();
	short getKeyState(short key);
};

#endif
