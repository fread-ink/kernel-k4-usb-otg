#ifndef __LINUX_REGULATOR_PAPYRUS_H_
#define __LINUX_REGULATOR_PAPYRUS_H_

enum {
    /* In alphabetical order */
    PAPYRUS_DISPLAY, /* virtual master enable */
    PAPYRUS_GVDD,
    PAPYRUS_GVEE,
    PAPYRUS_VCOM,
    PAPYRUS_VNEG,
    PAPYRUS_VPOS,
    PAPYRUS_TMST,
    PAPYRUS_NUM_REGULATORS,
};

/*
 * Declarations
 */
struct regulator_init_data;

struct papyrus_platform_data {
        unsigned int gvee_pwrup;
        unsigned int vneg_pwrup;
        unsigned int vpos_pwrup;
        unsigned int gvdd_pwrup;
        unsigned int gvdd_pwrdn;
        unsigned int vpos_pwrdn;
        unsigned int vneg_pwrdn;
        unsigned int gvee_pwrdn;
        int gpio_pmic_pwrgood;
        int gpio_pmic_vcom_ctrl;
        int gpio_pmic_wakeup;
        int gpio_pmic_intr;
        struct regulator_init_data *regulator_init;
};

#endif
