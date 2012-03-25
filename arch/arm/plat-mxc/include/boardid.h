/* TODO: Polish this or obtain original file from Amazon */
#define BOARD_ID_TEQUILA "TEQUILA"
#define BOARD_ID_TEQUILA_EVT1 BOARD_ID_TEQUILA
#define BOARD_ID_TEQUILA_EVT2 "TEQUILA_EVT2"

#define BOARD_IS__(board, ref) !strcmp(board, ref)

#define BOARD_IS_(board, ref, len) BOARD_IS__(ref, BOARD_ID_TEQUILA_EVT1) || BOARD_IS__(ref, BOARD_ID_TEQUILA_EVT2)
#define BOARD_REV_GREATER(board, ref) BOARD_IS__(ref, BOARD_ID_TEQUILA_EVT1)
#define BOARD_REV_GREATER_EQ(board, ref) BOARD_IS__(ref, BOARD_ID_TEQUILA_EVT1) || BOARD_IS__(ref, BOARD_ID_TEQUILA_EVT2)
