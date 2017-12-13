#include <stdint.h>
#include <time.h>
#include <string.h>
#include <pthread.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <ach.h>


#define AQUASHOKO_LEG_NUM               4
#define AQUASHOKO_LEG_JOINT_NUM         3
#define AQUASHOKO_CHAN_REF_NAME         "aquashoko_chan_ref"


typedef struct aquashoko_joint {
	double ref;
	double pos;
}__attribute((packed)) aquashoko_joint_t;

typedef struct aquashoko_leg {
	aquashoko_joint_t joint[AQUASHOKO_LEG_JOINT_NUM];
}__attribute((packed)) aquashoko_leg_t;

typedef struct aquashoko_ref {
	aquashoko_leg_t leg[AQUASHOKO_LEG_NUM];
}__attribute__((packed)) aquashoko_ref_t;
	





/* AquaShoko Structures */
aquashoko_ref_t aquashoko_ref;

/* Ach channels */
ach_channel_t aquashoko_chan_ref;  // reference channel

int aquashoko_set(int leg, int joint, double value)
{
  aquashoko_ref.leg[leg].joint[joint].pos = value;
  return 0;
}

double aquashoko_get(int leg, int joint)
{
  return aquashoko_ref.leg[leg].joint[joint].ref;
}

int aquashoko_pull()
{
  size_t fs;
  int ret = ach_get(&aquashoko_chan_ref, &aquashoko_ref, sizeof(aquashoko_ref), &fs, NULL, ACH_O_LAST);

  if(ACH_OK == ret) return 0;
  else return 1;
}

int aquashoko_put()
{
  int ret = ach_put(&aquashoko_chan_ref, &aquashoko_ref, sizeof(aquashoko_ref));

  if(ACH_OK == ret) return 0;
  else return 1;
}


int aquashoko_init()
{
  int ret = ach_open(&aquashoko_chan_ref, AQUASHOKO_CHAN_REF_NAME, NULL);
  memset(&aquashoko_ref, 0, sizeof(aquashoko_ref));

  if(ACH_OK == ret) return 0;
  else return 1;
}



