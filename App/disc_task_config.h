#ifndef DISC_TASK_CONFIG_H
#define DISC_TASK_CONFIG_H

/* Total timeout from task start, in milliseconds; do not reset per RFID ID.
 * Enforced by the PATH disc state machine, including camera preparation.
 */
#define DISC_TASK_TIMEOUT_MS    20000U

#endif /* DISC_TASK_CONFIG_H */
