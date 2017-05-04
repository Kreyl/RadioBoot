/*
 * shell.h
 *
 *  Created on: 25 окт. 2015 г.
 *      Author: Kreyl
 */

#ifndef KL_LIB_SHELL_H_
#define KL_LIB_SHELL_H_

#include <cstring>
#include "kl_lib.h"

#define CMD_BUF_SZ		99
#define DELIMITERS      " ,"

enum ProcessDataResult_t {pdrProceed, pdrNewCmd};

class Cmd_t {
private:
    char IString[CMD_BUF_SZ];
    uint32_t Cnt;
    bool Completed;
public:
    char *Name, *Token;
    ProcessDataResult_t PutChar(char c) {
        // Reset cmd if it was completed, and after that new char arrived
        if(Completed) {
            Completed = false;
            Cnt = 0;
        }
        // Process char
        if(c == '\b') { if(Cnt > 0) Cnt--; }    // do backspace
        else if((c == '\r') or (c == '\n')) {   // end of line, check if cmd completed
            if(Cnt != 0) {  // if cmd is not empty
                IString[Cnt] = 0; // End of string
                Name = strtok(IString, DELIMITERS);
                Completed = true;
                return pdrNewCmd;
            }
        }
        else if(Cnt < (CMD_BUF_SZ-1)) IString[Cnt++] = c;  // Add char if buffer not full
        return pdrProceed;
    }
    uint8_t GetNextTokenString() {
        Token = strtok(NULL, DELIMITERS);
        return (*Token == '\0')? FAILURE : OK;
    }
    uint8_t GetNextNumber(int32_t *POutput) {
        uint8_t r;
        if((r = GetNextTokenString()) == OK) {
            if(*Token == '\0') return EMPTY;
            char *p;
            *POutput = strtol(Token, &p, 0);
            return (*p == '\0')? OK : NOT_A_NUMBER;
        }
        else return r;
    }
    bool NameIs(const char *SCmd) { return (strcasecmp(Name, SCmd) == 0); }
    Cmd_t() {
        Cnt = 0;
        Completed = false;
        Name = nullptr;
        Token = nullptr;
    }
};

class Shell_t {
protected:
	thread_t *IPThd;
public:
	Cmd_t Cmd;
	void SignalCmdProcessed() {
	    chSysLock();
	    if(IPThd->p_state == CH_STATE_SUSPENDED) chSchReadyI(IPThd);
	    chSysUnlock();
	}

	virtual void Printf(const char *S, ...);
	void Reply(const char* CmdCode, int32_t Data) { Printf("%S,%d\r\n", CmdCode, Data); }
	void Ack(int32_t Result) { Printf("Ack %d\r\n", Result); }
};

#endif /* KL_LIB_SHELL_H_ */
