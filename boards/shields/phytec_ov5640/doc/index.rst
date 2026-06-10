.. _phytec_ov5640:

PHYTEC OV5640 Camera Module
###############################

Overview
********

This shield supports ov5640 camera modules are made specifically for and
provided together with phyboard-atlas boards.

Pins assignment of the NXP board-to-board 44-pin OV5640 camera module
======================================================================

+----------------------+--------------------+
| Camera Connector Pin | Function           |
+======================+====================+
| 1                    | GND                |
+----------------------+--------------------+
| 2                    | MIPI_CSI_DN0       |
+----------------------+--------------------+
| 3                    | MIPI_CSI_DP0       |
+----------------------+--------------------+
| 4                    | GND                |
+----------------------+--------------------+
| 5                    | MIPI_CSI_DN1       |
+----------------------+--------------------+
| 6                    | MIPI_CSI_DP1       |
+----------------------+--------------------+
| 7                    | GND                |
+----------------------+--------------------+
| 8                    | MIPI_CSI_CKN       |
+----------------------+--------------------+
| 9                    | MIPI_CSI_CKP       |
+----------------------+--------------------+
| 10                   | GND                |
+----------------------+--------------------+
| 11                   | CAM_IO0            |
+----------------------+--------------------+
| 12                   | CAM_IO1            |
+----------------------+--------------------+
| 13                   | LPI2C_SCL          |
+----------------------+--------------------+
| 14                   | LPI2C_SDA          |
+----------------------+--------------------+
| 15                   | VCC_CAM            |
+----------------------+--------------------+
| 16                   | GND                |
+----------------------+--------------------+
| 17                   | GND                |
+----------------------+--------------------+

Requirements
************

This shield can only be used with a board which provides rpi camera compatable
connector with MIPI CSI or DVP (parallel) interface where the pinouts are defined
as above.

Programming
***********

Set ``--shield phytec_ov5640`` when you invoke ``west build``. For example:

.. zephyr-app-commands::
   :zephyr-app: samples/drivers/video/capture
   :board: phyboard_atlas/mimxrt1176/cm7
   :shield: phytec_ov5640
   :goals: build

.. include:: ../../../nxp/common/board-footer.rst.inc
