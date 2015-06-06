#include "opencv2/core/core.hpp"
#include "opencv2/opencv.hpp"
#include <iostream>
#include <iomanip>
#include <ctime>
//http://www.cs.kent.edu/~farrell/sp/reference/multi-thread.html#thread_condvar
//http://www.cs.kent.edu/~ruttan/sysprog/lectures/multi-thread/line-count.c
#include <cstdio>             /* standard I/O routines                      */
#include <pthread.h>           /* pthread functions and data structures      */
#include <unistd.h>
#include <cstring>
#include <cstdlib>
#include <ctime>

// locals
#include "defines.h"
#include "detector.h"
#include "serial.h"
#include "brain.h"
#include "simulation.h"

using namespace std;

//Mat frame;

static void help() {
	cout << "Usage: ./detect [<OPT> <value>, ...]" << endl;
	cout << "\tavailable OPT:" << endl;
    cout << "\t\t" << OPT_NO_CAM << ", no following value" << endl;
	cout << "\t\t" << OPT_EXP << ", values: (0.0, 1.0]" << endl;
	cout << "\t\t" << OPT_ARDUINO_1
			<< "\tvalues: string to com port like /dev/ttyACM0" << endl;
	cout << "\t\t" << OPT_ARDUINO_2
			<< "\tvalues: string to com port like /dev/ttyACM1" << endl;
	cout << "\t\t" << OPT_WTC
			<< "\tvalues: string to com port like /dev/ttyUSB0" << endl;
	cout << "\t\t" << OPT_BAUDRATE << "\tvalues: 9600 .. 115200" << endl;
    cout << "\t\t" << OPT_IMAGE_LOG_TIME << "\tvalues number of images to skip: 1 .. ?" << endl;
}

/* global mutex for detector */
pthread_mutex_t detector_mutex;

/* global mutexes for serial communication */
pthread_mutex_t ard1_mutex;
pthread_mutex_t ard2_mutex;
pthread_mutex_t wtc_mutex;

/* global condition variable for our program. assignment initializes it. */
pthread_cond_t action_cond = PTHREAD_COND_INITIALIZER;

/* flag to denote if the user requested to cancel the operation in the middle */
/* 0 means 'no'. */
int cancel_operation = 0;
int camera_loop_running = 0;
bool simulation = 0;
bool video = 1;

// global objects
Serial *ard1 = NULL;
Serial *ard2 = NULL;
Serial *wtc = NULL;
Detector *detector = NULL;
Brain *brain = NULL;

// functions
void restore_coocked_mode(void* dummy);
void* read_user_input(void*);
void* camera_loop(void *data);
void* communication_loop(void *data);
void updateReadings(char* values, int len);
void sendInstruction(char* cmd, int len);

/*
 * function: restore_coocked_mode - restore normal screen mode.
 * algorithm: uses the 'stty' command to restore normal screen mode.
 *            serves as a cleanup function for the user input thread.
 * input: none.
 * output: none.
 */

void restore_coocked_mode(void* dummy) {
#ifdef DEBUG
	printf("restore_coocked_mode: before 'stty -raw echo'\n\r");
	fflush(stdout);
#endif /* DEBUG */
	system("stty -raw echo");
#ifdef DEBUG
	printf("restore_coocked_mode: after 'stty -raw echo'\n");
	fflush(stdout);
#endif /* DEBUG */
}

/*
 * function: read_user_input - read user input while long operation in progress.
 * algorithm: put screen in raw mode (without echo), to allow for unbuffered
 *            input.
 *            perform an endless loop of reading user input. If user
 *            pressed 'e', signal our condition variable and end the thread.
 * input: none.
 * output: none.
 */
void* read_user_input(void* data) {
    Brain *brain = (Brain*) data;
    
    if (!brain->getNumPorts()) {
        cout << "No serial connection for testing!!" << endl;
//        /* mark that there was a cancel request by the user */
//        cancel_operation = 1;
//        /* signify that we are done */
//        pthread_cond_signal(&action_cond);
//        pthread_exit(NULL);
    }
    
	int c;
    bool new_command = false;
    int forward = 455;
    int backward = 55;
    int halt = 255;
    char left = CMD_WHEELS_R;
    char right =CMD_WHEELS_L;

	/* register cleanup handler */
	pthread_cleanup_push(restore_coocked_mode, NULL);

    /* make sure we're in asynchronous cancelation mode so   */
    /* we can be canceled even when blocked on reading data. */
    pthread_setcanceltype(PTHREAD_CANCEL_ASYNCHRONOUS, NULL);

    /* put screen in raw data mode */
    system("stty raw -echo");

    /* "endless" loop - read data from the user.            */
    /* terminate the loop if we got a 'e', or are canceled. */
    while ((c = getchar()) != EOF) {
        brain->updateReadings();
        brain->printReadings();
        switch (c) {
        case 'e':
#ifdef DEBUG
            printf("\n\ngot a 'e'\n\n\r");
            fflush(stdout);
#endif /* DEBUG */
            /* mark that there was a cancel request by the user */
            cancel_operation = 1;
            /* signify that we are done */
            pthread_cond_signal(&action_cond);
            pthread_exit(NULL);
            break;
        case 'i': // go straight forward
                printf("go straight forward\n\r");
                brain->setCommand(left, forward);
                brain->setCommand(right, forward);
                new_command = true;
                break;
        case 'j': // turn left
                printf("turn left\n\r");
                brain->setCommand(left, backward);
                brain->setCommand(right, forward);
                new_command = true;
                break;
        case 'k': // go straight backward
                printf("go straight backward\n\r");
                brain->setCommand(left, backward);
                brain->setCommand(right, backward);
                new_command = true;
                break;
        case 'l': // turn right
                printf("turn right\n\r");
                brain->setCommand(left, forward);
                brain->setCommand(right, backward);
                new_command = true;
                break;
        case ' ': // turn right
                printf("halt\n\r");
                brain->setCommand(left, halt);
                brain->setCommand(right, halt);
                brain->setCommand(CMD_BRUSH, 500);
                new_command = true;
                break;
        case 'f': // spin brush
                printf("brush forward\n\r");
                brain->setCommand(CMD_BRUSH, 999);
                new_command = true;
                break;
        case 'r': // spin brush
                printf("brush backward\n\r");
                brain->setCommand(CMD_BRUSH, 0);
                new_command = true;
                break;
        case 'd': // spin brush
                printf("brush stop\n\r");
                brain->setCommand(CMD_BRUSH, 500);
                new_command = true;
                break;
        case 'a': // open gate
                printf("tail open \n\r");
                brain->setCommand(CMD_TAIL, 180);
                new_command = true;
                break;
        case 'A': // close gate
                printf("tail close \n\r");
                brain->setCommand(CMD_TAIL, 70);
                new_command = true;
                break;
        case 'x': // lift up
                printf("lift down \n\r");
                brain->setCommand(CMD_LIFT, VAL_LIFT_LOW);
                new_command = true;
                break;
        case 'X': // lower lift
                printf("lift up \n\r");
                brain->setCommand(CMD_LIFT, VAL_LIFT_HIGH);
                new_command = true;
                break;
        case 's': // toggle simulation
                printf("toggle simulation \n\r");
                simulation = !simulation;
                break;
                
        default:
            break;
        }
        if (new_command) {
            brain->sendInstructions();
            simulation = false;
            new_command = false;
        }
        fflush(stdout);
        usleep(COM_WAIT_TIME);
    }

    /* pop cleanup handler, while executing it, to restore cooked mode. */
    pthread_cleanup_pop(1);
	return 0;
}

void* camera_loop(void *data) {
	Detector *detector = (Detector*) data;
	double elapsed_secs = 1.0;
	clock_t t0 = clock();
	int counter = 0;
	camera_loop_running = 1;

	while (!cancel_operation) {

		pthread_mutex_lock(&detector_mutex);
		detector->detect(); // get a new frame from camera
		pthread_mutex_unlock(&detector_mutex);

		++counter;

		if (cancel_operation) {
			printf("CAMERA LOOP: operation canceled\n");
			break;
		}
		usleep(CAM_WAIT_TIME);

	}
	elapsed_secs = double(clock() - t0) / CLOCKS_PER_SEC;

	cout << "captured " << counter << " images in " << elapsed_secs << endl;

	pthread_exit(NULL);
}

void* communication_loop(void *data) {
	Serial *ser = (Serial*) data;
	pthread_mutex_t *mutex = (pthread_mutex_t*) ser->getMutex();

	while (!cancel_operation) {
		pthread_mutex_lock(mutex);
		ser->sread();
		if (ser->newWrite())
			ser->swrite();
		pthread_mutex_unlock(mutex);
//        fflush(stdout);
		usleep(COM_WAIT_TIME);
	}

	pthread_exit(NULL);
}

int main(int argc, char** args) {
    
//    char s[10] = {'\0'};
//    printf("%s\n", s);
//    sprintf(s, "%d", 10);
//    printf("%s\n", s);
//    sprintf(s, "%03d", 1000);
//    printf("%s\n", s);
//    printf("%d\n", atoi(s));
//    
//    return 0;

	char *port_ard1 = 0;
	char *port_ard2 = 0;
	char *port_wtc = 0;
	float exposure = 0.05;
	int baudrate = BAUDRATE;
    int interval = 0;
	char *argument;

    if (argc < 2) {
        help();
        return -1;
    }
	for (int i = 1; i < argc; i += 2) {
        if (0 == strcmp(args[i], OPT_NO_CAM)) {
            video = false;
            --i;
            continue;
        }
		if (i + 1 >= argc) {
			help();
			return -1;
		}
		argument = args[i];
		if (0 == strcmp(argument, OPT_EXP)) {
			exposure = atof(args[i + 1]);
			cout << "OPT exposure: " << exposure << endl;
		} else if (0 == strcmp(argument, OPT_ARDUINO_1)) {
			port_ard1 = args[i + 1];
			cout << "OPT port arduino 1: " << port_ard1 << endl;
		} else if (0 == strcmp(argument, OPT_ARDUINO_2)) {
			port_ard2 = args[i + 1];
			cout << "OPT port arduino 2: " << port_ard2 << endl;
		} else if (0 == strcmp(argument, OPT_WTC)) {
			port_wtc = args[i + 1];
			cout << "OPT port wtc: " << port_wtc << endl;
		} else if (0 == strcmp(argument, OPT_BAUDRATE)) {
			baudrate = atoi(args[i + 1]);
			cout << "OPT baudrate: " << baudrate << endl;
        } else if (0 == strcmp(argument, OPT_IMAGE_LOG_TIME)) {
            interval = atoi(args[i + 1]);
            cout << "OPT image interval: " << interval << endl;
		} else {
			help();
			return -1;
		}
	}

	int camnum = 1;

	// thread handles
	pthread_t thread_camera_loop; // 'handle' of camera-loop thread.
	pthread_t thread_communication_loop_ard1; // 'handle' of ard1 loop thread.
	pthread_t thread_communication_loop_ard2; // 'handle' of ard2-loop thread.
	pthread_t thread_communication_loop_wtc; // 'handle' of wtc-loop thread.
	pthread_t thread_user_input; /* 'handle' of user-input thread.           */

    if(video) {
        cout << "trying to connect to camera " << camnum << endl;
        /* ----- create needed objects ---- */
//        detector = new Detector(camnum, exposure, 240, 320, &detector_mutex);
        detector = new Detector(camnum, exposure, 480, 640, &detector_mutex);

        // threading stuff
        if (detector->isReady()) {
            if (pthread_mutex_init(&detector_mutex, NULL) != 0) {
                printf("\n detector_mutex init failed\n");
                return 1;
            }
            pthread_create(&thread_camera_loop, NULL, camera_loop, detector);
        } else {
            delete (detector);
            detector = NULL;
            cout << "running without camera" << endl;
        }
    } else {
        detector = NULL;
        cout << "running without camera" << endl;
    }
    int max_read = 2 * NUM_READINGS * READING_SIZE;
    int max_write = 200;
    
	// collect serials to hand over to brain
    Serial *ser_collection[3] = {NULL, NULL, NULL};
    int ser_map[3] = {ID_NONE, ID_NONE, ID_NONE};

	if (port_ard1 != 0) {
		if (pthread_mutex_init(&ard1_mutex, NULL) != 0) {
			printf("\n ard1_mutex init failed\n");
			return 1;
		}
		ard1 = new Serial(port_ard1, baudrate, max_read, max_write,
				&ard1_mutex);
		if (!ard1->isOpened())
			return 1;
		pthread_create(&thread_communication_loop_ard1, NULL,
				communication_loop, ard1);
		ser_collection[ard1->getID()] = ard1;
		ser_map[ID_ARD1] = ard1->getID();
	}
	if (port_ard2 != 0) {
		if (pthread_mutex_init(&ard2_mutex, NULL) != 0) {
			printf("\n ard2_mutex init failed\n");
			return 1;
		}
		ard2 = new Serial(port_ard2, baudrate, max_read, max_write,
				&ard2_mutex);
		if (!ard2->isOpened())
			return 1;
		pthread_create(&thread_communication_loop_ard2, NULL,
				communication_loop, ard2);
		ser_collection[ard2->getID()] = ard2;
		ser_map[ID_ARD2] = ard2->getID();
	}
	if (port_wtc != 0) {
		if (pthread_mutex_init(&wtc_mutex, NULL) != 0) {
			printf("\n wtc_mutex init failed\n");
			return 1;
		}
		wtc = new Serial(port_wtc, baudrate, max_read, max_write, &wtc_mutex);
		if (!wtc->isOpened())
			return 1;
		pthread_create(&thread_communication_loop_wtc, NULL, communication_loop,
				wtc);
		ser_collection[wtc->getID()] = wtc;
		ser_map[ID_WTC] = wtc->getID();
	}
    
    cout << "creating brain with " << Serial::getClassCount() << " connections" << endl;
    
    // create brain object
    brain = new Brain(ser_collection, ser_map, Serial::getClassCount(), detector);

	/* spawn the user-reading thread */
	pthread_create(&thread_user_input, NULL, read_user_input, brain);

//	if (camera_loop_running) namedWindow("preview");
	Mat cframe;
    int pic_count = 0;
    string folder_pic = "pics";
    
    Simulation* mySim = new Simulation(brain);
    
	for (int dummy = 0;;) {
#ifdef DEBUG
		if (camera_loop_running) {
			pthread_mutex_lock (&detector_mutex);
			cframe = detector->getFrame().clone();
			pthread_mutex_unlock(&detector_mutex);
			imshow("preview", cframe);
    //        imshow("mask", mask * 255);
			char c = (char) waitKey(1);
            if (interval)
            {
                if (dummy == 0) {
                    stringstream ss;
                    ss << folder_pic << "/" << setfill('0') << std::setw(4) << (++pic_count) << ".png";
                    cout << "writing image " << ss.str() << imwrite(ss.str(), cframe) << endl;
                }
                dummy = (dummy + 1) % interval;
//                else
//                    cout << "here" << (t1 - clock())<< " " << CLOCKS_PER_SEC << endl;
            }
            
		}

#endif
        
        if (simulation) {
            mySim->loop();
        }
        
		/* check if we were signaled due to user operation        */
		/* cancelling, or because the line-counting was finished. */
		if (cancel_operation) {
			/* we join it to make sure it restores normal */
			/* screen mode before we print out.           */
			cout << "waiting for 1" << endl << "\r";
			pthread_join(thread_user_input, NULL);
			cout << "waiting for 2" << endl;
			if (detector != NULL) pthread_join(thread_camera_loop, NULL);
			cout << "waiting for 3" << endl;
			if (ard1 != NULL) pthread_join(thread_communication_loop_ard1, NULL);
			cout << "waiting for 4" << endl;
			if (ard2 != NULL) pthread_join(thread_communication_loop_ard2, NULL);
			cout << "waiting for 5" << endl;
			if (wtc != NULL) pthread_join(thread_communication_loop_wtc, NULL);
			printf("operation canceled\n");
			break;
		}
		usleep(COM_WAIT_TIME);

	}

	// clean up
	if (detector != NULL) {
		pthread_mutex_destroy(&detector_mutex);
		delete (detector);
	}
	if (ard1 != NULL) {
		pthread_mutex_destroy(&ard1_mutex);
		delete (ard1);
	}
	if (ard2 != NULL) {
		pthread_mutex_destroy(&ard2_mutex);
		delete (ard2);
	}
	if (wtc != NULL) {
		pthread_mutex_destroy(&wtc_mutex);
		delete (wtc);
	}
//    *camera_loop(&exposure);
//    pthread_exit(NULL);
	return 0;
}

