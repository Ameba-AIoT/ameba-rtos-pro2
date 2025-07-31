Example Description

This example describes how to use SRAM retention for Standby.

NOTE:
    retention variables must be linked to ".retention.data" section which is in SRAM retention memory region.
    SLP_GTIMER must be used to make SRAM retention work.
    This example will continue to sleep and wakup. SLP_AON_TIMER is used to wakeup after 5s. The execution times will be counted and saved by SRAM retention.
