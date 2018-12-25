#ifndef __FC_RC_H
#define __FC_RC_H

float getSetpointRate(int axis);
float getThrottlePIDAttenuation(void);
void generateThrottleCurve(void);
void updateRcCommands(void);
void processRcCommand(void);

#endif	// __FC_RC_H
