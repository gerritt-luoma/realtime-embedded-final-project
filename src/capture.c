/*
 *
 *  Adapted by Sam Siewert for use with UVC web cameras and Bt878 frame
 *  grabber NTSC cameras to acquire digital video from a source,
 *  time-stamp each frame acquired, save to a PGM or PPM file.
 *
 *  The original code adapted was open source from V4L2 API and had the
 *  following use and incorporation policy:
 *
 *  This program can be used and distributed without restrictions.
 *
 *      This program is provided with the V4L2 API
 * see http://linuxtv.org/docs.php for more information
 */
#define _GNU_SOURCE

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <assert.h>
#include <stdint.h>
#include <sys/uio.h>

/* Used for getting cmdline args */
#include <getopt.h>

#include <unistd.h>
#include <errno.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <sys/time.h>
#include <sys/mman.h>
#include <sys/ioctl.h>

#include <linux/videodev2.h>

#include <time.h>

// needed for threading and message queue
#include <pthread.h>
#include <mqueue.h>
#include <unistd.h>
#include <semaphore.h>
#include <sched.h>

// needed for logging to system journal
#include <syslog.h>
// needed for setting message queue limits
#include <sys/resource.h>
#include <sys/time.h>
#include <sys/sysinfo.h>
#include <signal.h>

/* Definitions for frame handling */
#define START_UP_FRAMES (8)                 /* Number of frames to always discard on startup */
#define LAST_FRAMES (1)                     /* Additional frame to ensure 1801 frames are collected */
#define WINDOW_SIZE 2                       /* Size of ping pong buffer for saving current and last frame */
#define CAPTURE_FRAMES (1800 + LAST_FRAMES) /* Total number of frames to capture */
#define MY_CLOCK_TYPE CLOCK_MONOTONIC_RAW   /* Desired clock for timing */
#define TIME_EXECUTION 1                    /* Define for toggling timing behavior */

/* Variables for timing the execution of the software */
static double fstart = 0.0, fstop = 0.0;
struct timespec start_time_val;
double start_realtime;

#ifdef TIME_EXECUTION
    /* Additional optional timing vars */
    double capture_post_time, capture_period;
    double process_post_time, process_period;
#endif

/* Frame size definitions */
#define HRES 320
#define VRES 240
#define FRAME_W HRES
#define FRAME_H VRES
#define RGB_SZ (FRAME_W * FRAME_H * 3)
#define GRAY_SZ (FRAME_W * FRAME_H)

/* Threading definitions */
#define NUM_CPU_CORES (4)
#define NUM_THREADS (2)
#define TRUE (1)
#define FALSE (0)
#define ERROR (-1)


/* Thread communication defines */
#define MAX_MSG_SIZE (sizeof(captured_frame_t))
#define CAP2DET_MQ "/cap_to_det_mq"
#define DET2SAVE_MQ "/det_to_save_mq"

/* Shared MQ attributes */
struct mq_attr mq_attr;
/* Global MQ handle for image cap to detection */
mqd_t cap2det_mq_sender;
/* Global control flags for aborting and signaling individual services. */
int abort_test = FALSE, abort_image_capture = FALSE, abort_image_detect = FALSE;
/* Semaphores for each scheduled service */
sem_t sem_image_capture, sem_image_tick_detect;

/* Structure for thread params */
typedef struct
{
    int threadIdx;
} threadParams_t;


/* Structure for saving frames in different formats with a timestamp */
typedef struct
{
    struct timespec ts;
    unsigned char rgb[RGB_SZ];
    unsigned char gray[GRAY_SZ];
} __attribute__((packed)) captured_frame_t;


/* Format is used by a number of functions, so made as a global variable */
static struct v4l2_format fmt;

/* Structure for V4L2 buffers */
struct buffer
{
    void *start;
    size_t length;
};

/* Capture global variables */
static char *dev_name;
static int fd = -1;
struct buffer *buffers;
static unsigned int n_buffers;

/* Run time features available */
/* Save images in grayscale instead of RGB */
int ten_hz_program          = FALSE;
int save_grayscale          = FALSE;
int apply_alpha_beta_filter = FALSE;

/* Variables for the sequencer */
static timer_t timer_1;
static struct itimerspec itime = {{1, 0}, {1, 0}};
static struct itimerspec last_itime;
static unsigned long long seqCnt = 0;

/* Frame processing variables */
int framecnt = -START_UP_FRAMES;       /* Always discard the first N frames before sending to the detect thread */
unsigned char bigbuffer[(1280 * 960)]; /* Buffer for converting from YUYV to RGB */

/* Circular window used for detecting frame and tick transitions */
static captured_frame_t capture_window[WINDOW_SIZE];
static unsigned char last_tick_buf[GRAY_SZ];
int last_tick_valid = 0;

/* Sequencer and service prototypes */
void Sequencer(int id);              /* Sequencer running at 60Hz */
void *image_capture(void *arg);      /* Image capture running at 30 or 4Hz */
void *image_tick_detect(void *arg);  /* Tick detection running at 20 or 4Hz */
void* save_data_to_flash(void *arg); /* Best effort service that runs until all images are saved */

/* Helper functions */
void alpha_beta_filter(captured_frame_t* image);      /* Applies an alpha-beta filter to image. brights and contrasts */
double realtime(struct timespec *tsptr);              /* Used for converting timespec to double */
static void push_ppm_to_detect_thread(const void *p); /* Helper for pushing to the detect frame since the cap handle is global */

/* Camera and frame processing functions */
static int read_frame(void);
static void process_image(const void *p, int size);
void yuv2rgb(int y, int u, int v, unsigned char *r, unsigned char *g, unsigned char *b);
void rgb_to_gray(unsigned char *dst, const unsigned char *src, size_t image_size_in_bytes);
static void init_mmap(void);
static void init_device(void);
static void open_device(void);
static void start_capturing(void);
static void image_capture_main_loop(void);
static void stop_capturing(void);
static void close_device(void);
static void uninit_device(void);

/* Detection and saving helper functions */
static inline double percent_diff(const unsigned char *a, const unsigned char *b, size_t image_size_in_bytes);
static void save_frame(captured_frame_t *image, const char *prefix, unsigned frame_count, int log);

/* Used for converting timespec to double */
double realtime(struct timespec *tsptr)
{
    return ((double)(tsptr->tv_sec) + (((double)tsptr->tv_nsec) / 1000000000.0));
}

/* System calls */
static void errno_exit(const char *s)
{
    fprintf(stderr, "%s error %d, %s\n", s, errno, strerror(errno));
    exit(EXIT_FAILURE);
}

static int xioctl(int fh, int request, void *arg)
{
    int r;

    do
    {
        r = ioctl(fh, request, arg);
    } while (-1 == r && EINTR == errno);

    return r;
}


/* Applies an alpha-beta filter to image. brights and contrasts */
void alpha_beta_filter(captured_frame_t* image)
{
    #define PIXEL_IDX ((i*col*chan)+(j*chan)+k)
    #define MAX_PIXEL_VALUE (255)
    double alpha=1.25;  unsigned char beta=25;
    unsigned row=HRES, col=VRES, chan=3, pix;
    // apply filter to entire image
    for(int i=0; i < row; i++)
    {
        for(int j=0; j < col; j++)
        {
            for(int k=0; k < chan; k++)
            {
                //Clamp the new pixel value to be within 255
                // Apply alpha (contrast) and beta (brightness) adjustments
                image->rgb[PIXEL_IDX] = (pix=(unsigned)((image->rgb[PIXEL_IDX])*alpha)+beta) > MAX_PIXEL_VALUE ? MAX_PIXEL_VALUE : pix;
            }
        }
    }
}

/* Helper for pushing to the detect frame since the cap handle is global */
static void push_ppm_to_detect_thread(const void *p)
{
    if (mq_send(cap2det_mq_sender, p, MAX_MSG_SIZE, 30) == ERROR)
    {
        perror("mq_send");
    }
}

// This is probably the most acceptable conversion from camera YUYV to RGB
//
// Wikipedia has a good discussion on the details of various conversions and cites good references:
// http://en.wikipedia.org/wiki/YUV
//
// Also http://www.fourcc.org/yuv.php
//
// What's not clear without knowing more about the camera in question is how often U & V are sampled compared
// to Y.
//
// E.g. YUV444, which is equivalent to RGB, where both require 3 bytes for each pixel
//      YUV422, which we assume here, where there are 2 bytes for each pixel, with two Y samples for one U & V,
//              or as the name implies, 4Y and 2 UV pairs
//      YUV420, where for every 4 Ys, there is a single UV pair, 1.5 bytes for each pixel or 36 bytes for 24 pixels
void yuv2rgb(int y, int u, int v, unsigned char *r, unsigned char *g, unsigned char *b)
{
    int r1, g1, b1;

    // replaces floating point coefficients
    int c = y - 16, d = u - 128, e = v - 128;

    // Conversion that avoids floating point
    r1 = (298 * c + 409 * e + 128) >> 8;
    g1 = (298 * c - 100 * d - 208 * e + 128) >> 8;
    b1 = (298 * c + 516 * d + 128) >> 8;

    // Computed values may need clipping.
    if (r1 > 255)
    {
        r1 = 255;
    }
    else if (r1 < 0)
    {
        r1 = 0;
    }

    if (g1 > 255)
    {
        g1 = 255;
    }
    else if (g1 < 0)
    {
        g1 = 0;
    }

    if (b1 > 255)
    {
        b1 = 255;
    }
    else if (b1 < 0)
    {
        b1 = 0;
    }

    *r = r1;
    *g = g1;
    *b = b1;
}

/* Convert RGB to grayscale using weighted values */
void rgb_to_gray(unsigned char *dst, const unsigned char *src, size_t image_size_in_bytes)
{
    for (size_t i = 0; i < image_size_in_bytes; ++i)
    {
        unsigned char r = src[3 * i];
        unsigned char g = src[3 * i + 1];
        unsigned char b = src[3 * i + 2];
        dst[i] = (unsigned char)(0.299 * r + 0.587 * g + 0.114 * b);
    }
}

/* Transforms image as necessary and grabs a real time timestamp of when the frame was saved */
static void process_image(const void *p, int size)
{
    int i, j;
    int y_temp, y2_temp, u_temp, v_temp;
    unsigned char *pptr = (unsigned char *)p;

    // Save real time frame time to image header
    struct timespec frame_capture_time;
    clock_gettime(CLOCK_REALTIME, &frame_capture_time);
    memcpy(bigbuffer, &frame_capture_time, sizeof(frame_capture_time));
    int starting_offset = sizeof(frame_capture_time);

    // Increment total number of captured frames
    framecnt++;

    // Converts the image to RGB if necessary and sends to the tick detection thread
    if (fmt.fmt.pix.pixelformat == V4L2_PIX_FMT_YUYV)
    {
        // Pixels are YU and YV alternating, so YUYV which is 4 bytes
        // We want RGB, so RGBRGB which is 6 bytes
        for (i = 0, j = starting_offset; i < size; i += 4, j += 6)
        {
            y_temp = (int)pptr[i];
            u_temp = (int)pptr[i + 1];
            y2_temp = (int)pptr[i + 2];
            v_temp = (int)pptr[i + 3];
            yuv2rgb(y_temp, u_temp, v_temp, &bigbuffer[j], &bigbuffer[j + 1], &bigbuffer[j + 2]);
            yuv2rgb(y2_temp, u_temp, v_temp, &bigbuffer[j + 3], &bigbuffer[j + 4], &bigbuffer[j + 5]);
        }

        // Only push ppm to next thread if the startup period has been crossed
        if (framecnt > 0)
        {
            push_ppm_to_detect_thread(bigbuffer);
        }
    }
    else if ((fmt.fmt.pix.pixelformat == V4L2_PIX_FMT_RGB24) && (framecnt > 0))
    {
        push_ppm_to_detect_thread(p);
    }


    fflush(stderr);
    fflush(stdout);
}

static int read_frame(void)
{
    struct v4l2_buffer buf;

    buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    buf.memory = V4L2_MEMORY_MMAP;
    // Dequeue buffer with captured frame
    if (-1 == xioctl(fd, VIDIOC_DQBUF, &buf))
    {
        switch (errno)
        {
            case EAGAIN:
            {
                return 0;
            }
            case EIO:
            {
                /* Could ignore EIO, but drivers should only set for serious errors, although some set for
                non-fatal errors too. */
                return 0;
            }
            default:
            {
                printf("mmap failure\n");
                errno_exit("VIDIOC_DQBUF");
            }
        }
    }

    /* Panic if index is out of range */
    assert(buf.index < n_buffers);

    /* Process the image to RGB and send to detection thread */
    process_image(buffers[buf.index].start, buf.bytesused);

    /* Enqueue the buffer into the capture queue */
    if (-1 == xioctl(fd, VIDIOC_QBUF, &buf))
    {
        errno_exit("VIDIOC_QBUF");
    }

    return 1;
}

static void image_capture_main_loop(void)
{
    while (!abort_image_capture)
    {
        /* Wait until sequencer releases this thread */
        sem_wait(&sem_image_capture);

        #ifdef TIME_EXECUTION
            struct timespec start;
            clock_gettime(MY_CLOCK_TYPE, &start);
            syslog(LOG_INFO, "SEQ CAPTURE START %.6lf", realtime(&start));
        #endif

        /* Loop as long as it takes to read the frame */
        for (;;)
        {
            fd_set fds;
            struct timeval tv;
            int r;

            FD_ZERO(&fds);
            FD_SET(fd, &fds);

            /* Timeout. */
            tv.tv_sec  = 2;
            tv.tv_usec = 0;
            /* Wait for data to be available */
            r = select(fd + 1, &fds, NULL, NULL, &tv);
            /* If there is an interrupt error retry the read */
            if (-1 == r)
            {
                if (EINTR == errno)
                {
                    continue;
                }
                errno_exit("select");
            }

            if (read_frame())
            {
                break;
            }
        }

        #ifdef TIME_EXECUTION
            struct timespec end;
            clock_gettime(MY_CLOCK_TYPE, &end);
            syslog(LOG_INFO, "SEQ CAPTURE END %.6lf", realtime(&end));
            syslog(LOG_INFO, "SEQ CAPTURE DUR %.6lf", realtime(&end) - realtime(&start));
            if (capture_period <= (realtime(&end) - realtime(&start)))
            {
                syslog(LOG_ERR, "Error: Possible capture deadline miss");
            }
        #endif
    }
}

static void stop_capturing(void)
{
    enum v4l2_buf_type type;

    /* Stop capturing */
    type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    if (-1 == xioctl(fd, VIDIOC_STREAMOFF, &type))
    {
        errno_exit("VIDIOC_STREAMOFF");
    }
}

static void start_capturing(void)
{
    unsigned int i;
    enum v4l2_buf_type type;

    /* Ensure each buffer is correctly configured and queued into the buffer */
    for (i = 0; i < n_buffers; ++i)
    {
        struct v4l2_buffer buf;

        buf.type   = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        buf.memory = V4L2_MEMORY_MMAP;
        buf.index  = i;

        if (-1 == xioctl(fd, VIDIOC_QBUF, &buf))
        {
            errno_exit("VIDIOC_QBUF");
        }
    }

    /* Start capturing */
    type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    if (-1 == xioctl(fd, VIDIOC_STREAMON, &type))
    {
        errno_exit("VIDIOC_STREAMON");
    }
}

static void uninit_device(void)
{
    unsigned int i;

    for (i = 0; i < n_buffers; ++i)
    {
        if (-1 == munmap(buffers[i].start, buffers[i].length))
        {
            errno_exit("munmap");
        }
    }

    free(buffers);
}

static void init_mmap(void)
{
    struct v4l2_requestbuffers req;

    /* Request 6 video capture buffers */
    req.count = 6;
    req.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    req.memory = V4L2_MEMORY_MMAP;

    if (-1 == xioctl(fd, VIDIOC_REQBUFS, &req))
    {
        if (EINVAL == errno)
        {
            fprintf(stderr, "%s does not support "
                            "memory mapping\n",
                    dev_name);
            exit(EXIT_FAILURE);
        }
        else
        {
            errno_exit("VIDIOC_REQBUFS");
        }
    }

    if (req.count < 2)
    {
        fprintf(stderr, "Insufficient buffer memory on %s\n", dev_name);
        exit(EXIT_FAILURE);
    }

    /* Allocate all of the available buffers */
    buffers = calloc(req.count, sizeof(*buffers));

    if (!buffers)
    {
        fprintf(stderr, "Out of memory\n");
        exit(EXIT_FAILURE);
    }

    /* Initialize each v4l2 buffer with the correct settings */
    for (n_buffers = 0; n_buffers < req.count; ++n_buffers)
    {
        struct v4l2_buffer buf;

        buf.type   = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        buf.memory = V4L2_MEMORY_MMAP;
        buf.index  = n_buffers;

        if (-1 == xioctl(fd, VIDIOC_QUERYBUF, &buf))
            errno_exit("VIDIOC_QUERYBUF");

        buffers[n_buffers].length = buf.length;
        buffers[n_buffers].start  = mmap(NULL /* start anywhere */,
                 buf.length,
                 PROT_READ | PROT_WRITE /* required */,
                 MAP_SHARED /* recommended */,
                 fd, buf.m.offset);

        if (MAP_FAILED == buffers[n_buffers].start)
        {
            errno_exit("mmap");
        }
    }
}

static void init_device(void)
{
    struct v4l2_capability cap;
    struct v4l2_cropcap cropcap;
    struct v4l2_crop crop;
    unsigned int min;

    /* Ensure device is v4l2 compatible */
    if (-1 == xioctl(fd, VIDIOC_QUERYCAP, &cap))
    {
        if (EINVAL == errno)
        {
            fprintf(stderr, "%s is no V4L2 device\n",
                    dev_name);
            exit(EXIT_FAILURE);
        }
        else
        {
            errno_exit("VIDIOC_QUERYCAP");
        }
    }

    /* Ensure device has video capture capabilities */
    if (!(cap.capabilities & V4L2_CAP_VIDEO_CAPTURE))
    {
        fprintf(stderr, "%s is no video capture device\n",
                dev_name);
        exit(EXIT_FAILURE);
    }

    /* Ensure device has streaming capabilities */
    if (!(cap.capabilities & V4L2_CAP_STREAMING))
    {
        fprintf(stderr, "%s does not support streaming i/o\n",
                dev_name);
        exit(EXIT_FAILURE);
    }

    /* Set cropping */
    cropcap.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    if (0 == xioctl(fd, VIDIOC_CROPCAP, &cropcap))
    {
        crop.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        crop.c = cropcap.defrect; /* reset to default */

        xioctl(fd, VIDIOC_S_CROP, &crop);
    }

    fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;

    fmt.fmt.pix.width = HRES;
    fmt.fmt.pix.height = VRES;
    fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_YUYV;
    fmt.fmt.pix.field = V4L2_FIELD_NONE;

    if (-1 == xioctl(fd, VIDIOC_S_FMT, &fmt))
        errno_exit("VIDIOC_S_FMT");

    /* Ensure data sizes are more than the needed size */
    min = fmt.fmt.pix.width * 2;
    if (fmt.fmt.pix.bytesperline < min)
    {
        fmt.fmt.pix.bytesperline = min;
    }

    min = fmt.fmt.pix.bytesperline * fmt.fmt.pix.height;
    if (fmt.fmt.pix.sizeimage < min)
    {
        fmt.fmt.pix.sizeimage = min;
    }

    init_mmap();
}

static void close_device(void)
{
    if (-1 == close(fd))
        errno_exit("close");

    fd = -1;
}

static void open_device(void)
{
    struct stat st;
    // check device exists
    if (-1 == stat(dev_name, &st))
    {
        fprintf(stderr, "Cannot identify '%s': %d, %s\n",
                dev_name, errno, strerror(errno));
        exit(EXIT_FAILURE);
    }
    // ensure device is a char device
    if (!S_ISCHR(st.st_mode))
    {
        fprintf(stderr, "%s is no device\n", dev_name);
        exit(EXIT_FAILURE);
    }
    // open device for reading and writing
    fd = open(dev_name, O_RDWR /* required */ | O_NONBLOCK, 0);

    if (-1 == fd)
    {
        fprintf(stderr, "Cannot open '%s': %d, %s\n",
                dev_name, errno, strerror(errno));
        exit(EXIT_FAILURE);
    }
}

void *image_capture(void *arg)
{
    // open queue to talk to thread that will save picture in flash
    cap2det_mq_sender = mq_open(CAP2DET_MQ, O_CREAT | O_RDWR, S_IRWXU, &mq_attr);

    // initialization of V4L2
    open_device();             // open the video device
    init_device();             // set up the video device and allocate buffers
    start_capturing();         // start streaming image frames
    image_capture_main_loop(); // enter capture loop
    stop_capturing();          // stop streaming video frames

    printf("Total capture time=%lf, for %d frames, %lf FPS\n", (fstop - fstart), CAPTURE_FRAMES, ((double)CAPTURE_FRAMES / (fstop - fstart)));

    uninit_device(); // clean up allocated resources
    close_device();  // close the video device
    fprintf(stderr, "\n");

    return NULL;
}

// absolute‑difference percentage
static inline double percent_diff(const unsigned char *a, const unsigned char *b, size_t image_size_in_bytes)
{
    unsigned int sum = 0;
    unsigned int noise_threshold = 30;
    for (size_t i = 0; i < image_size_in_bytes; ++i)
    {
        unsigned char diff = abs(a[i] - b[i]);
        if(diff < noise_threshold)
        {
            diff = 0;
        }
        sum += diff;
    }
    return (double)sum / (image_size_in_bytes * 255.0) * 100.0;
}

static void save_frame(captured_frame_t *image, const char *prefix, unsigned frame_count, int log)
{
    char name[128];
    char header[128];
    struct iovec iov[2];
    struct timespec time_image_taken = image->ts;
    int version;
    unsigned char *image_data;
    int data_length;
    // determine image format based on user arguments
    if(save_grayscale)
    {
        snprintf(name, sizeof(name), "%s%08u.pgm", prefix, frame_count);
        version = 5;
        image_data = image->gray;
        data_length = GRAY_SZ;
    }
    else
    {
        snprintf(name, sizeof(name), "%s%08u.ppm", prefix, frame_count);
        version = 6;
        image_data = image->rgb;
        data_length = RGB_SZ;
    }

    int fd = open(name, O_WRONLY | O_CREAT | O_TRUNC, 0666);
    if (fd < 0)
    {
        perror("Failed to open file");
        return;
    }

    int hlen = snprintf(header, sizeof(header),
                        "P%d\n#%010ld sec %03ld msec\n%d %d\n255\n", version,
                        time_image_taken.tv_sec, time_image_taken.tv_nsec / 1000000, FRAME_W, FRAME_H);

    iov[0].iov_base = header;
    iov[0].iov_len = hlen;
    iov[1].iov_base = (void *)image_data;
    iov[1].iov_len = data_length;

    if (writev(fd, iov, 2) != (ssize_t)(hlen + data_length))
    {
        perror("Failed to write file");
    }

    if(log)
    {
        syslog(LOG_INFO, "[Course #:4][Final Project][Frame Count: %u][Image Capture Start Time: %ld.%09ld]", frame_count, image->ts.tv_sec, image->ts.tv_nsec);
    }

    close(fd);
}

void *image_tick_detect(void *arg)
{

    last_tick_valid = 0;

    mqd_t cap2det_mq = mq_open(CAP2DET_MQ, O_CREAT|O_RDWR|O_NONBLOCK, S_IRWXU, &mq_attr);
    if (cap2det_mq == (mqd_t)-1)
    {
        perror("mq_open");
        return NULL;
    }

    mqd_t det2save_mq = mq_open(DET2SAVE_MQ, O_CREAT|O_RDWR|O_NONBLOCK, S_IRWXU, &mq_attr);
    if (det2save_mq == (mqd_t)-1)
    {
        perror("mq_open");
        return NULL;
    }

    unsigned int frames_rx = 0, discard_cnt = 0, num_to_discard = ten_hz_program ? 45 : 8;
    unsigned int ticks_detected = 0;
    // next location of incoming image
    int head = 0;

    while (ticks_detected < CAPTURE_FRAMES && !abort_image_detect)
    {
        if (sem_wait(&sem_image_tick_detect) != 0)
        {
            perror("sem_wait");
            break;
        }

        #ifdef TIME_EXECUTION
            struct timespec start;
            clock_gettime(MY_CLOCK_TYPE, &start);
            syslog(LOG_INFO, "SEQ DETECT START %.6lf", realtime(&start));
        #endif

        // Drain everything that is already in the queue. The queue was opened
        // with O_NONBLOCK, so mq_receive returns -1/EAGAIN when it is empty.
        while (1)
        {
            captured_frame_t *current_frame = &capture_window[head];

            ssize_t rc = mq_receive(cap2det_mq, (char *)&current_frame->ts, MAX_MSG_SIZE, NULL);
            if (rc < 0)
            {
                // queue empty. pend again
                if (errno == EAGAIN)
                {
                    break;
                }
                perror("mq_receive");
                continue;
            }

            // warm‑up. dont care about these frames
            if (discard_cnt < num_to_discard)
            {
                ++discard_cnt;
                continue;
            }

            ++frames_rx;

            /* Convert frame to grayscale regardless of if it will be used for processing*/
            rgb_to_gray(current_frame->gray, current_frame->rgb, FRAME_W * FRAME_H);

            if (frames_rx >= WINDOW_SIZE)
            {
                /* Use a ping pong buffer to hold current and last frame */
                captured_frame_t *last_frame = &capture_window[(head + 1) % WINDOW_SIZE];

                /* Delta between current image and previous image */
                double diff_to_previous = percent_diff(last_frame->gray, current_frame->gray, FRAME_W * FRAME_H);
                syslog(LOG_INFO, "frames_rx: %d diff_to_previous: %f\n",frames_rx, diff_to_previous);

                const double STABLE_THR = ten_hz_program ? 0.04 : 0.004;
                const double CHANGE_THR = ten_hz_program ? 0.13 : 0.05;

                /* If we have started detecting frames and the last two frames are stable */
                if (diff_to_previous <= STABLE_THR)
                {
                    if (last_tick_valid)
                    {
                        /* Diff current frame from last valid tick */
                        double current_frame_diff = percent_diff(last_tick_buf, current_frame->gray, FRAME_W * FRAME_H);

                        /* Use the frame with the largest difference */
                        if (current_frame_diff >= CHANGE_THR)
                        {
                            syslog(LOG_INFO, "found image current_frame_diff=%lf", current_frame_diff);

                            /* Write "detected" time to frame timespec */
                            struct timespec now;
                            clock_gettime(CLOCK_REALTIME, &now);
                            current_frame->ts = now;

                            /* Send to saving thread and copy this frame to the last tick buffer */
                            if (mq_send(det2save_mq, (char*)&current_frame->ts, MAX_MSG_SIZE, 30) == ERROR)
                            {
                                perror("mq_send");
                            }

                            memcpy(last_tick_buf, current_frame->gray, GRAY_SZ);
                            ++ticks_detected;

                            /* Output to stdout every 10 seconds */
                            if (ticks_detected % 100 == 0)
                            {
                                printf("Detected %d/1801 frames\n", ticks_detected);
                            }

                        }
                        else
                        {
                            syslog(LOG_INFO, "frame isn't different enough: current_frame_diff=%lf", current_frame_diff);
                        }
                    }
                    else
                    {
                        syslog(LOG_INFO, "saving first frame");

                        /* Write "detected" time to frame timespec */
                        struct timespec now;
                        clock_gettime(CLOCK_REALTIME, &now);
                        current_frame->ts = now;
                        fstart = realtime(&now);

                        /* Send to saving thread and copy this frame to the last tick buffer */
                        if (mq_send(det2save_mq, (char*)&current_frame->ts, MAX_MSG_SIZE, 30) == ERROR)
                        {
                            perror("mq_send");
                        }
                        memcpy(last_tick_buf, current_frame->gray, GRAY_SZ);
                        ++ticks_detected;
                        last_tick_valid = 1;
                    }
                }
                else
                {
                    syslog(LOG_INFO, "frames aren't stable: diff_to_previous=%lf", diff_to_previous);
                }
            }
            else
            {
                syslog(LOG_INFO, "not enough frames in buffer yet");
            }
            // advance window
            head = (head + 1) % WINDOW_SIZE;
        }

        #ifdef TIME_EXECUTION
            struct timespec end;
            clock_gettime(MY_CLOCK_TYPE, &end);
            syslog(LOG_INFO, "SEQ DETECT END %.6lf", realtime(&end));
            syslog(LOG_INFO, "SEQ DETECT DUR %.6lf", realtime(&end) - realtime(&start));
            if (process_period <= (realtime(&end) - realtime(&start)))
            {
                syslog(LOG_ERR, "Error: Possible deadline miss on frame processing task");
            }
        #endif
    }

    struct timespec stop;
    clock_gettime(CLOCK_REALTIME, &stop);
    fstop = realtime(&stop);

    abort_test = TRUE;
    return NULL;
}

void* save_data_to_flash(void *arg)
{
    int frames_saved = 0;
    captured_frame_t new_image;

    mqd_t det2save_mq = mq_open(DET2SAVE_MQ, O_CREAT|O_RDWR, S_IRWXU, &mq_attr);
    if (det2save_mq == (mqd_t)-1)
    {
        perror("mq_open");
        return NULL;
    }

    while(1)
    {
        if (mq_receive(det2save_mq, (char *)&new_image.ts, MAX_MSG_SIZE, NULL) < 0)
        {
            perror("mq_receive");
            continue;
        }

        #ifdef TIME_EXECUTION
            struct timespec start;
            clock_gettime(MY_CLOCK_TYPE, &start);
            syslog(LOG_INFO, "SEQ SAVE START %.6lf", realtime(&start));
        #endif

        /* Apply the alpha beta filter if requested */
        if (apply_alpha_beta_filter)
        {
            alpha_beta_filter(&new_image);
        }

        /* Re-convert to grayscale again in case the alpha beta filter was applied */
        if(save_grayscale)
        {
            rgb_to_gray(new_image.gray, new_image.rgb, FRAME_W * FRAME_H);
        }

        /* Saves the image to .ppm or to .pgm */
        save_frame(&new_image, "tick", ++frames_saved, TRUE);

        #ifdef TIME_EXECUTION
            struct timespec end;
            clock_gettime(MY_CLOCK_TYPE, &end);
            syslog(LOG_INFO, "SEQ SAVE END %.6lf", realtime(&end));
            syslog(LOG_INFO, "SEQ SAVE DUR %.6lf", realtime(&end) - realtime(&start));
        #endif

        if(frames_saved >= CAPTURE_FRAMES)
        {
            break;
        }
    }

    return NULL;
}

void Sequencer(int id)
{
    int flags = 0;

    // received interval timer signal

    seqCnt++;

    // Release each service at a sub-rate of the generic sequencer rate

    // Service_1 = RT_MAX-1	@ 4 or 30 Hz depending on run type
    int service_1_period = ten_hz_program ? 2 : 15;
    if ((seqCnt % service_1_period) == 0)
    {
        #ifdef TIME_EXECUTION
            struct timespec now;
            clock_gettime(MY_CLOCK_TYPE, &now);
            syslog(LOG_INFO, "SEQ CAPTURE POST %ld.%09ld", now.tv_sec, now.tv_nsec);
            capture_post_time = realtime(&now);
        #endif
        sem_post(&sem_image_capture);
    }

    // Service_2 = RT_MAX-2	@ 4 or 20 Hz depending on run type Hz
    int service_2_period = ten_hz_program ? 3 : 15;
    if ((seqCnt % service_2_period) == 0)
    {
        #ifdef TIME_EXECUTION
            struct timespec now;
            clock_gettime(MY_CLOCK_TYPE, &now);
            syslog(LOG_INFO, "SEQ DETECT POST %ld.%09ld", now.tv_sec, now.tv_nsec);
            process_post_time = realtime(&now);
        #endif
        sem_post(&sem_image_tick_detect);
    }

    if (abort_test)
    {
        // disable interval timer
        itime.it_interval.tv_sec = 0;
        itime.it_interval.tv_nsec = 0;
        itime.it_value.tv_sec = 0;
        itime.it_value.tv_nsec = 0;
        timer_settime(timer_1, flags, &itime, &last_itime);

        // shutdown all services
        sem_post(&sem_image_capture);
        sem_post(&sem_image_tick_detect);

        abort_image_capture = TRUE;
        abort_image_detect = TRUE;
    }
}

int main(int argc, char **argv)
{
    if(argc <= 1)
    {
        printf("usage: capture <ten_hz_program> <save_grayscale> <apply_alpha_beta_filter>");
        exit(1);
    }

    /* Save the selected required cmdline inputs */
    ten_hz_program          = atoi(argv[1]);
    save_grayscale          = atoi(argv[2]);
    apply_alpha_beta_filter = atoi(argv[3]);

    /* Default to video0 */
    dev_name = "/dev/video0";

    /* set inf limit for queue to allow full message to go over queue */
    struct rlimit rlim;
    rlim.rlim_cur = RLIM_INFINITY;
    rlim.rlim_max = RLIM_INFINITY;

    if (setrlimit(RLIMIT_MSGQUEUE, &rlim) == -1)
    {
        perror("setrlimit");
        return 1;
    }
    int rc = 0;

    /* Setup common message q attributes */
    mq_attr.mq_maxmsg = 200;
    mq_attr.mq_msgsize = MAX_MSG_SIZE;

    mq_attr.mq_flags = 0;

    struct timespec current_time_val, current_time_res;
    double current_realtime, current_realtime_res;

    int i, flags = 0;

    /* CPU affinity variables for binding threads to specific cores. */
    cpu_set_t threadcpu;
    cpu_set_t allcpuset;
    int cpuidx;

    /* declare thread parms and init parms */
    pthread_t threads[NUM_THREADS];
    threadParams_t threadParams[NUM_THREADS];
    pthread_attr_t rt_sched_attr[NUM_THREADS];
    int rt_max_prio, rt_min_prio;

    /* Declare schedule parms for real time execution */
    struct sched_param rt_param[NUM_THREADS];
    struct sched_param main_param;

    /* Output startup message dictating time and clock resolution */
    printf("Starting Tick Detection Demo\n");
    clock_gettime(MY_CLOCK_TYPE, &start_time_val);
    start_realtime = realtime(&start_time_val);
    clock_gettime(MY_CLOCK_TYPE, &current_time_val);
    current_realtime = realtime(&current_time_val);
    clock_getres(MY_CLOCK_TYPE, &current_time_res);
    current_realtime_res = realtime(&current_time_res);
    printf("START Tick Detection @ sec=%6.9lf with resolution %6.9lf\n", (current_realtime - start_realtime), current_realtime_res);

    // Log hardware platform info: number of processors configured and available.
    printf("System has %d processors configured and %d available.\n", get_nprocs_conf(), get_nprocs());

    // try to use all cpus
    CPU_ZERO(&allcpuset);
    for (i = 0; i < NUM_CPU_CORES; i++)
    {
        CPU_SET(i, &allcpuset);
    }

    printf("Using CPUS=%d from total available.\n", CPU_COUNT(&allcpuset));

    /* initialize the sequencer semaphores */
    if (sem_init(&sem_image_capture, 0, 0))
    {
        printf("Failed to initialize image capture semaphore\n");
        exit(-1);
    }
    if (sem_init(&sem_image_tick_detect, 0, 0))
    {
        printf("Failed to initialize image tick detect semaphore\n");
        exit(-1);
    }
    /* Don't need a semaphore for saving the image because it is blocking on receiving the image */

    /* Setup real time processing (SCHED_FIFO) in the main thread */
    rt_max_prio = sched_get_priority_max(SCHED_FIFO);
    rt_min_prio = sched_get_priority_min(SCHED_FIFO);

    rc = sched_getparam(getpid(), &main_param);
    main_param.sched_priority = rt_max_prio;
    rc = sched_setscheduler(getpid(), SCHED_FIFO, &main_param);
    if (rc < 0)
    {
        perror("main_param");
    }

    printf("rt_max_prio=%d\n", rt_max_prio);
    printf("rt_min_prio=%d\n", rt_min_prio);

    /* For each service thread, set CPU affinity, scheduling parameters, and priority */
    for (i = 0; i < NUM_THREADS; i++)
    {
        /* Run both main threads on core 0 */
        CPU_ZERO(&threadcpu);
        cpuidx = (0);
        CPU_SET(cpuidx, &threadcpu);

        rc = pthread_attr_init(&rt_sched_attr[i]);
        /* Explicitly set scheduling parameters (do not inherit default settings) */
        rc = pthread_attr_setinheritsched(&rt_sched_attr[i], PTHREAD_EXPLICIT_SCHED);
        /* Set real-time policy SCHED_FIFO for the thread */
        rc = pthread_attr_setschedpolicy(&rt_sched_attr[i], SCHED_FIFO);
        /* Set CPU affinity for the thread */
        rc = pthread_attr_setaffinity_np(&rt_sched_attr[i], sizeof(cpu_set_t), &threadcpu);

        pthread_attr_setschedparam(&rt_sched_attr[i], &rt_param[i]);

        threadParams[i].threadIdx = i;
    }

    printf("Service threads will run on %d CPU cores\n", CPU_COUNT(&threadcpu));

    #ifdef TIME_EXECUTION
        capture_period = ten_hz_program ? (1.0 / 30.0) : (1.0 / 4.0);
        process_period = ten_hz_program ? (1.0 / 20.0) : (1.0 / 4.0);
    #endif /* TIME_EXECUTION */

    /* Image Capture Service = RT_MAX-1	@ 4 or 30 Hz */
    rt_param[0].sched_priority = rt_max_prio - 1;
    pthread_attr_setschedparam(&rt_sched_attr[0], &rt_param[0]);
    rc = pthread_create(&threads[0],       // pointer to thread descriptor
                        &rt_sched_attr[0], // use specific attributes
                        //(void *)0,               // default attributes
                        image_capture,             // thread function entry point
                        (void *)&(threadParams[0]) // parameters to pass in
    );
    if (rc < 0)
    {
        perror("pthread_create for service 1");
    }
    else
    {
        printf("pthread_create successful for service 1\n");
    }

    /* Tick Detect Service = RT_MAX-2	@ 4 or 20 Hz */
    rt_param[1].sched_priority = rt_max_prio - 2;
    pthread_attr_setschedparam(&rt_sched_attr[1], &rt_param[1]);
    rc = pthread_create(&threads[1], &rt_sched_attr[1], image_tick_detect, (void *)&(threadParams[1]));
    if (rc < 0)
    {
        perror("pthread_create for service 2");
    }
    else
    {
        printf("pthread_create successful for service 2\n");
    }


    /* Create the flash save thread */
    pthread_t thread_flash_save;

    printf("trying to start flash thread\n");

    /* Create the flash saving thread */
    if (pthread_create(&thread_flash_save, NULL, save_data_to_flash, 0) != 0)
    {
        perror("Failed to create thread");
    }
    else
    {
        printf("pthread_create successful for flash service\n");
    }

    /* Sleep to give the threads time to wait on semaphores/mqs */
    sleep(2);

    /* Set CPU affinity to CPU 1 (the second CPU core) */
    cpu_set_t cpuset;
    CPU_ZERO(&cpuset);
    CPU_SET(1, &cpuset);
    if (pthread_setaffinity_np(thread_flash_save, sizeof(cpu_set_t), &cpuset) != 0) {
        perror("pthread_setaffinity_np");
        return 1;
    }

    printf("Start sequencer\n");
    /* Sequencer = RT_MAX	@ 60 Hz */
    //
    /* Set up to signal SIGALRM if timer expires */
    timer_create(CLOCK_REALTIME, NULL, &timer_1);

    signal(SIGALRM, (void (*)())Sequencer);

    /* Arm the interval timer */
    itime.it_interval.tv_sec = 0;
    itime.it_interval.tv_nsec = 16666666;
    itime.it_value.tv_sec = 0;
    itime.it_value.tv_nsec = 16666666;

    timer_settime(timer_1, flags, &itime, &last_itime);

    for (i = 0; i < NUM_THREADS; i++)
    {
        if (pthread_join(threads[i], NULL) < 0)
            perror("main pthread_join");
        else
            printf("joined thread %d\n", i);
    }

    if (pthread_join(thread_flash_save, NULL) < 0) {
        perror("main pthread_join");
    }
    else
    {
        printf("joined flash thread\n");
    }

    return 0;
}
