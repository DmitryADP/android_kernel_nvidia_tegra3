int qc750_get_board_strap(void);
int qc750_get_hw_ramcode(void);

enum qc750_board_version
{
        QC750_BOARD_VER_UNKNOW = 0,
        QC750_BOARD_VER_A00    = 0x7,
        QC750_BOARD_VER_B00    = 0x5,
        QC750_BOARD_VER_C00    = 0x3,
        QC750_BOARD_VER_D00    = 0x1,
};
