/*
 ************************************************************************
 * @file shell_driver.h
 * @author Boston O'Neill, 45798737
 * @date 04/04/22
 * @brief Include file for shell driver.
 **********************************************************************
 */

#ifndef SHELL_DRIVER_H
#define SHELL_DRIVER_H

/*Define shell thread stack size*/
#define SHELL_STACK_SIZE 8192

/* Add thread to include for main.c*/
void shell_entry(void *arg1, void *arg2, void *arg3);

#endif /*SHELL_DRIVER_H*/