/*
 * PixyCameraI2C.cpp
 *
 *  Created on: Jan 24, 2017
 *      Author: Gaelhawks
 */

#include "WPILib.h"
#include "TalonXIX_main.h"
#include "PixyCameraI2C.h"


PixyJunk::PixyJunk(int address)
{
	 i2cAddress = address;
     pixyi2c = new I2C(I2C::kMXP, i2cAddress);

     // declare the object data variables
     LocalReset();
}

void PixyJunk::LocalReset()
{
     isBlockDetected = false;
     blockCount = 0;
 	 ignoredTarget = false;
 	 skipStart = false;
 	 syncFailed = false;
	 lastPixyWord = 0xffff;
 	 printCount = 0;
 	 wordCount = 0;

 	 leftIndex = 0;
 	 rightIndex = 0;
 	 topIndex = 0;
 	 bottomIndex = 0;

 	 isHorizontal = false;
}

// the remainder of this snippet should be placed in a loop where the data is also used.
// a while loop is suggested where the loop exits when the target is identified or a break button is
// depressed on the OI

void PixyJunk::UpdateDash()
{
    SmartDashboard::PutBoolean("Target Detected", TargetDetected());
    SmartDashboard::PutNumber("Number of blocks Detected", GetNumBlocks());
}


bool PixyJunk::StartTracking() //checks whether if it is start of the normal frame, CC frame, or the data is out of sync
{
	 int targetLoopCount = 0;
	 bool searchStartWord = true;
	 lastPixyWord = 0xffff;

	 //printf("SEARCHING for START WORD...\n");
	 while(searchStartWord)
	 {
		 currPixyWord = GetWord(); //This it the function right underneath
		 //printf("%d Current Word %x\n", wordCount, currPixyWord);
		 //wordCount++;
		 if (currPixyWord == 0 && lastPixyWord == 0)
		 {
			 isBlockDetected = false;
			 syncFailed = true;
			 printf("Pixy: sync FAILURE...\n");
			 return false;
		 }
		 else if (currPixyWord==PIXY_START_WORD && lastPixyWord==PIXY_START_WORD)
		 {
			 blockType = NORMAL_BLOCK;
			 isBlockDetected = true;
			 //syncFailed = false;
			 //printf("Pixy: sync GOOD...\n");
			 return true;
		 }
		 else if (currPixyWord==PIXY_START_WORD_CC && lastPixyWord==PIXY_START_WORD)
		 {
			 blockType = CC_BLOCK;
			 isBlockDetected = true;
			 //syncFailed = false;
			 //printf("Pixy: sync CC block...\n");
			 return true;
		 }
		 else if (currPixyWord==PIXY_START_WORDX) //when byte recieved was 0x55aa instead of otherway around, the code syncs the byte
		 {
			 //printf("Pixy: reorder");
			 GetByte(); // resync
		 }
		 else
		 {
			 targetLoopCount++;
			 if(targetLoopCount > 50)
			 {
				 printf("Pixy: sync TIMEOUT...\n");
				 isBlockDetected = false;
				 searchStartWord = false;
				 return false;
			 }
		 }

		 lastPixyWord = currPixyWord;
	 }
	 //printf("Pixy: StartTracking() return false\n");
	 return false;
}

uint16_t PixyJunk::GetWord() //Getting two Bytes from Pixy (The full information)
{
	unsigned char buffer[2] = {0, 0};

	pixyi2c->ReadOnly(2, buffer);
	return (buffer[1] << 8) | buffer[0]; //shift buffer[1] by 8 bits and add( | is bitwise or) buffer[0] to it
}

uint8_t PixyJunk::GetByte()//gets a byte
{
	unsigned char buffer[1] = {0};

	pixyi2c->ReadOnly(1, buffer);
	return buffer[0];
}

int PixyJunk::GetBlocks(int maxBlocks)
{
	 blocks[0] = {0}; //resets the array - clears out data from previous reading
	 uint8_t blockIndex;
	 uint16_t pixyWord, checksum, sum;
	 Block *block;

	 blockCount = 0;
	 //printf("\nSTART of GETBLOCKS\n");

	 if (skipStart)
	 {
		 skipStart = false;
		 isBlockDetected = true;
	 }
	 else
	 {  //when computer has not seen 0xaa55 (starting frame)
		if (StartTracking() == false)
		{
			isBlockDetected = false;
			//printf("No start frame\n");
			return -1;
		}
	 }
	 //lastPixyWord = 0xffff;


	 //for(blockCount=0; blockCount<maxBlocks && blockCount<PIXY_MAXIMUM_ARRAYSIZE;)
	 while(true)//Read until we break out at the end of the block
	 {
		checksum = GetWord();
		//printf("%d Check Sum %x\n", wordCount, checksum);
		//wordCount++;
		if (checksum == PIXY_START_WORD) // we've reached the beginning of the next frame - checking for 0xaa55
		{
		   skipStart = true; //starts this function
		   blockType = NORMAL_BLOCK;
		   if (syncFailed)
		   {
			   //printf("sync failed, skip block\n");
			   syncFailed = false;
			   return -1;
		   }
		   //printf("skip, block count %d\n", blockCount);
		   return blockCount;
		}
		else if (checksum == PIXY_START_WORD_CC) //we've reached the beginning of the next frame - checking for 0xaa56
		{
			skipStart = true;
			blockType = CC_BLOCK;
		    if (syncFailed)
		    {
		    	//printf("sync failed, skip block\n");
			    syncFailed = false;
			    return -1;
		    }
			//printf("CC skip\n");
			return blockCount;
		}
		else if (checksum == 0)
		{
			printf("Checksum=0?? Return blockCount %d\n", blockCount);
		    return blockCount;
		}

		//if (blockCount>blockArraySize)
		   //resize();

		//block = blocks + blockCount;
		if(blockCount < MAX_BLOCKS)
		{
			block = &(blocks[blockCount]);
		}
		else
		{
			block = NULL;
		    printf("block is NULL\n");
		}

		//The loop indexes through the entire block of data received for that target and stores it in the block structure
		for (blockIndex = 0, sum = 0; blockIndex < sizeof(Block)/sizeof(uint16_t); blockIndex++)
		{
		   if (blockType == NORMAL_BLOCK && blockIndex >= 5) // skip --if not an CC block, no need to consider angle
		   {
			  block->angle = 0;
			  break;
		   }
		   pixyWord = GetWord();
		   sum += pixyWord; //sum = w + sum
		   //printf("%d Pixy Word %x	Block Count %d\n", wordCount, pixyWord, blockCount);
		   //wordCount++;
		   if (block != NULL)
		   {
			   *((uint16_t *)block + blockIndex) = pixyWord; //converts block to integer value
		   }
		}
		if (checksum == sum)
		{
		   blockCount++;
		   //printf("Next block is %d\n", blockCount);
		}
		else
		   printf("Pixy: checksum error\n");

		pixyWord = GetWord(); //when this is start of the frame
		//printf("%d Pixy Word2 %x	Block Count %d\n", wordCount, pixyWord, blockCount);
		//wordCount++;
		if (pixyWord == PIXY_START_WORD)
		{
		   //lastPixyWord = pixyWord; // allow it to see the double startword for a new frame
		   blockType = NORMAL_BLOCK;
		}
		else if (pixyWord == PIXY_START_WORD_CC)
		   blockType = CC_BLOCK;
		else
		{
		   syncFailed = false;
		   //printf("successful return %d blocks\n", blockCount);
		   return blockCount;
		}
	 }
	 printf("End: No Targets\n");
	 return 0;
}

void PixyJunk::ZeroOutTarget()
{
    target.signature = 0;
    target.xPosition = 0;
    target.yPosition = 0;
    target.targetWidth = 0;
    target.targetHeight = 0;
    target.targetGap = 0;
    target.numBlocks = 0;
    target.topY = 0;
    target.bottomY = 0;
    target.leftX = 0;
    target.rightX = 0;
    target.isHorizontal = false;
}

int PixyJunk::AnalyzeTarget()
{
	printCount++;
	int leftWidthCorner = 0;
	int rightWidthCorner = 0;
	int topHeightCorner = 0;
	int bottomHeightCorner = 0;

	int prevBlockCount = blockCount;  // save previous block count in case we get an intermittent null block


	//ZeroOutTarget();
	GetBlocks(MAX_BLOCKS);

	// If we get only one zero block, ignore, it is most likely a processing error and the previous target is returned.
	//But if we get two or more in a row then we return no target is found
    //if (blockCount == 0 && prevBlockCount != blockCount)

    if (!ignoredTarget && (prevBlockCount != blockCount))
	{
    	blockCount = prevBlockCount;
    	ignoredTarget = true;
    	//printf("%d ignore intermittent block count change\n", printCount);
    	return blockCount;
	}

	ZeroOutTarget();
	ignoredTarget = false;
	if(blocks[0].width > blocks[0].height)
	{
		isHorizontal = true;
	}
	else
	{
		isHorizontal = false;
	}
	target.isHorizontal = isHorizontal;

	switch(blockCount)
	{
		case 0:
			return blockCount;
			break;

		case 1:
			//By definition, the only block is left, right, top, and bottom, there is only one block
			leftIndex = 0;
			rightIndex = 0;
			topIndex = 0;
			bottomIndex = 0;

			target.xPosition = blocks[0].x;
			target.yPosition = blocks[0].y;
			break;

		default:
			if(blocks[0].x < blocks[1].x)
			{
				leftIndex = 0;
				rightIndex = 1;
			}
			else
			{
				leftIndex = 1;
				rightIndex = 0;
			}

			if(blocks[0].y < blocks[1].y)
			{
				topIndex = 1;
				bottomIndex = 0;
			}
			else
			{
				topIndex = 0;
				bottomIndex = 1;
			}

			if(isHorizontal)
			{
				int gapTopCorner;
				int gapBottomCorner;

				gapTopCorner = blocks[topIndex].y - (blocks[topIndex].height/2);
				gapBottomCorner = blocks[bottomIndex].y + (blocks[bottomIndex].height/2);
				target.targetGap = gapTopCorner - gapBottomCorner;
			}
			else
			{
				int gapLeftCorner;
				int gapRightCorner;

				gapLeftCorner = blocks[leftIndex].x - (blocks[leftIndex].width/2);
				gapRightCorner = blocks[rightIndex].x + (blocks[rightIndex].width/2);
				target.targetGap = gapRightCorner - gapLeftCorner;
			}

			target.xPosition = (blocks[0].x + blocks[1].x)/2;
			target.yPosition = (blocks[0].y + blocks[1].y)/2;
			break;

	}

	leftWidthCorner = (blocks[leftIndex].x - blocks[leftIndex].width/2);
	rightWidthCorner = (blocks[rightIndex].x + blocks[rightIndex].width/2);
	target.targetWidth = rightWidthCorner - leftWidthCorner;

	topHeightCorner = (blocks[topIndex].y + blocks[topIndex].height/2);
	bottomHeightCorner = (blocks[bottomIndex].y - blocks[bottomIndex].height/2);
	target.targetHeight = topHeightCorner - bottomHeightCorner;

    target.topY = (target.yPosition + target.targetHeight/2);
    target.bottomY = (target.yPosition - target.targetHeight/2);
    target.leftX = (target.xPosition - target.targetWidth/2);
    target.rightX = (target.xPosition + target.targetWidth/2);

	target.numBlocks = blockCount;
	return target.numBlocks;
}

void PixyJunk::PrintBlock(int index)
{
	if(blockCount == 0)
	{
		printf("NO TARGET!!! :(\n");
		return;
	}
	if(index < 0)
	{
		//printf("Composite Target:  numBlock: %d  x: %d y: %d width: %d height: %d Gap: %d  orientation: %s \n", target.numBlocks, target.xPosition, target.yPosition, target.targetWidth, target.targetHeight, target.targetGap, target.isHorizontal?"Horizontal":"Vertical");
		//printf("Composite Target:   numBlock: %d         x: %d          width: %d  \n", target.numBlocks, target.xPosition,  target.targetWidth);
		printf("%d Composite Target:   numBlock: %d      top: %d     center: %d   \n", printCount, target.numBlocks, target.leftX,  target.yPosition);
	}
	else if(index < GetNumBlocks() && index >= 0)
	{
		 if(isBlockDetected)
		{
			if (blocks[index].signature>PIXY_MAX_SIGNATURE) // color code! (CC)
			{
			  printf("CC block! numblock: %d  sig: %X (%d decimal) x: %d y: %d width: %d height: %d angle %d\n", blockCount, blocks[index].signature, blocks[index].signature, blocks[index].x, blocks[index].y, blocks[index].width, blocks[index].height, blocks[index].angle);
			}
			else // regular block.  Note, angle is always zero, so no need to print
			{
			  //printf("Block Num: %d numBlock: %d sig: %d x: %d y: %d width: %d height: %d\n", index, blockCount, blocks[index].signature, blocks[index].x, blocks[index].y, blocks[index].width, blocks[index].height); //prints out data to console
			  printf("Target:       x: %d          y: %d  \n", blocks[index].x, blocks[index].y);
			}
			//Serial.print(buf);
		}
		else
		{
			printf("No target detected (index: %d)\n", index);
		}
	}
	else
	{
		printf("Invalid Target\n");
	}
}

int PixyJunk::GetNumBlocks()
{
	return blockCount;
}

Block* PixyJunk::GetTarget(int index)
{
	if(index <= GetNumBlocks())
	{
		return &(blocks[index]);
	}
	else
	{
		return NULL;
	}
}

Block* PixyJunk::GetTopTarget(void)
{
	return &(blocks[topIndex]);
}

Block* PixyJunk::GetBottomTarget(void)
{
	return &(blocks[bottomIndex]);
}

Block* PixyJunk::GetLeftTarget(void)
{
	return &(blocks[leftIndex]);
}

Block* PixyJunk::GetRightTarget(void)
{
	return &(blocks[rightIndex]);
}

CompositeBlock* PixyJunk::GetCompositeTarget()
{
	return &(target);
}

bool PixyJunk::IsHorizontal()
{
	return isHorizontal;
}

bool PixyJunk::TargetDetected()
{
	if(blockCount < 1)
	{
		return false;
	}
	else
	{
		return true;
	}
}

void PixyJunk::ResyncCamera()
{
	printf("Resyncing...");
	double cameraStartTime = GetTime();
	skipStart = false;
	delete(pixyi2c);
    pixyi2c = new I2C(I2C::kMXP, i2cAddress);
    LocalReset();
	double cameraEndTime = GetTime();
	printf("Resync Time: %f\n", cameraEndTime - cameraStartTime);
}

