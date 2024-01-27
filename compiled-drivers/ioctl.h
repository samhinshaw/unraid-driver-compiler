/* Definition for IOCTL */
#define IOCTL_SET_CUSTOM		_IOW(0xD0, 11, int)
#define IOCTL_GET_CUSTOM		_IOR(0xD0, 12, int)
#define IOCTL_GET_PRODUCT		_IOR(0xD0, 13, int)
#define IOCTL_SET_PARAMETER		_IOW(0xD0, 14, int)

#define IOCTL_GPIO_DIR			_IOW(0xD0, 15, int)
#define IOCTL_GPIO_STATUS		_IOW(0xD0, 16, int)
#define IOCTL_GPIO_OUTPUT		_IOR(0xD0, 17, int)

#define DLL_MASK		0xFFFFF00F
#define DLM_MASK		0xFFF00FFF
#define SAMPLE_MASK		0xFFFFFFF0
#define BASE_CLOCK_MASK		0xF00FFFFF
