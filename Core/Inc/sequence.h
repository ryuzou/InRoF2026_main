#ifndef SEQUENCE_H
#define SEQUENCE_H

#ifdef __cplusplus
extern "C" {
#endif

void Sequence_WaitForRobotCommand(void);
void Sequence_IssueMoveToRP2(void);
void Sequence_CalibrateHeadingWithTsd10YWall(void);
void Sequence_CallibrateRP1(void);
void Sequence_CallibrateRP2(void);
void Sequence_CollectBalls(void);
void Sequence_PlaceStoredBalls(void);

#ifdef __cplusplus
}
#endif

#endif /* SEQUENCE_H */
