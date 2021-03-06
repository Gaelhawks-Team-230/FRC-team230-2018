/*
 * PixyCameraI2C.h
 *
 *  Created on: Jan 24, 2017
 *      Author: Gaelhawks
 */

#ifndef SRC_PIXYCAMERAI2C_H_
#define SRC_PIXYCAMERAI2C_H_

#include "WPILib.h"
#include "Common.h"

// set the number of bytes to get from the pixycam each read cycle.  The pixycam outputs 14 byte blocks
// of data with an extra 2 bytes between frames per Object Block Format Figure
#define MAX_I2C_BYTES				(4)
#define PIXY_INITIAL_ARRAYSIZE      30
#define PIXY_MAXIMUM_ARRAYSIZE      130
#define PIXY_START_WORD             0xaa55 //for regular color recognition
#define PIXY_START_WORD_CC          0xaa56 //for color code - angle rotation recognition
#define PIXY_START_WORDX            0x55aa //regular color another way around
#define PIXY_MAX_SIGNATURE          7
#define PIXY_DEFAULT_ARGVAL         0xffff
#define MAX_BLOCKS					2


enum BlockType
{
   NORMAL_BLOCK, //normal color recognition
   CC_BLOCK     //color-code(change in angle) recognition
};

struct Block
{
	uint16_t signature; //Identification number for your object - you could set it in the pixymon
	uint16_t x; //0 - 320
	uint16_t y; //0 - 200
	uint16_t width;
	uint16_t height;
	uint16_t angle;
};

struct CompositeBlock
{
    uint16_t signature;
    uint16_t xPosition;
    uint16_t yPosition;
    uint16_t targetWidth;
    uint16_t targetHeight;
    uint16_t targetGap;
    uint16_t numBlocks;
    uint16_t topY;
    uint16_t bottomY;
    uint16_t leftX;
    uint16_t rightX;
    bool isHorizontal;
};


class PixyJunk
{
private:
    frc::I2C *pixyi2c;
// declare a byte array to store the data from the camera
    //uint8_t pixyValues[MAX_I2C_BYTES];
    //uint8_t testValues[2];


    BlockType blockType;// it is the enum on the top
    uint16_t currPixyWord;
	uint16_t lastPixyWord;
    bool  skipStart;   //skips to check 0xaa55, which is byte that tells pixy it is start of new frame
    bool syncFailed;
    int blockCount; //How many signatured objects are there?
    Block blocks[PIXY_MAXIMUM_ARRAYSIZE]; //array that stores blockCount array
    CompositeBlock target;
    bool ignoredTarget;
    int printCount;
    int i2cAddress;
    int wordCount;

	int leftIndex;
	int rightIndex;
	int topIndex;
	int bottomIndex;

    bool isBlockDetected;

    bool isHorizontal;

    bool StartTracking(void);
    uint16_t GetWord(void);
    uint8_t GetByte(void);


public:
    PixyJunk(int address);
    void LocalReset(void);
    void UpdateDash(void);
    int GetBlocks(int maxBlocks);
    int AnalyzeTarget(void);
    void PrintBlock(int index);
    void ZeroOutTarget(void);
    void ResyncCamera(void);
    int GetNumBlocks(void);
    Block* GetTarget(int index);
    Block* GetTopTarget(void);
    Block* GetBottomTarget(void);
    Block* GetLeftTarget(void);
    Block* GetRightTarget(void);
    CompositeBlock* GetCompositeTarget(void);
    bool IsHorizontal();
    bool TargetDetected();
};


#endif /* SRC_PIXYCAMERAI2C_H_ */
