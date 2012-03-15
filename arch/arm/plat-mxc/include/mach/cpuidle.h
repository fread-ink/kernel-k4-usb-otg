#include <linux/cpuidle.h>

struct imx_cpuidle_state {
       unsigned short state_number;
       struct cpuidle_state *state;
};
