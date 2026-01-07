using namespace std;

#include "lib/cxxopts/include/cxxopts.hpp"

#include "lib/serialib/lib/serialib.h"
#include "StateMachine/SerialStateMachine.h"

#include "StateMachine/Nodes/SendStringNode.h"
#include "StateMachine/Nodes/NopNode.h"
#include "StateMachine/Nodes/IncrementingNode.h"
#include "StateMachine/Nodes/StreamNode.h"
#include "StateMachine/Nodes/StringListNode.h"

#include <stdio.h>
#include <fstream>
#include <string>

#define SERIAL_PORT "/dev/ttyACM0"
#define BANK_SIZE 0x4000  // 16KB per bank
#define DEFAULT_NUM_BANKS 128  // Default number of banks to read (2MB)

int checkBankExtension(const char* str) {
    string s(str);
    s = s.substr(s.length()-7);
    string s2 = s.substr(0, 5);
    if(s2.compare(string(".bank")) == 0) {
        return strtol(s.substr(5).c_str(), nullptr, 16);
    }
    return -1;
}

string getBankNum(const char* str) {
    string s(str);
    return s.substr(s.length()-2);
}

// Read mode - dump flash contents to file
int readFlashToFile(serialib& serial, const string& filename, int startBank, int numBanks) {
    char buf[4096];
    char lineBuf[256];
    char cmdBuf[64];
    
    printf("Reading banks %d-%d (%d KB) to %s...\n", startBank, startBank + numBanks - 1, numBanks * 16, filename.c_str());
    
    // Wait for "Ready!" - consume all startup messages
    serial.readString(lineBuf, '!', 256, 5000);  // Hello world!
    serial.readString(lineBuf, '!', 256, 5000);  // Ready!
    printf("Device ready\n");
    
    // Open output file
    ofstream outFile(filename.c_str(), ios::binary);
    if (!outFile.is_open()) {
        printf("Error: Could not open output file %s\n", filename.c_str());
        return -1;
    }
    
    unsigned long totalBytes = 0;
    int endBank = startBank + numBanks;
    
    for (int bank = startBank; bank < endBank; bank++) {
        // Shift to bank
        sprintf(cmdBuf, "shift %x\r", bank);
        serial.writeString(cmdBuf);
        // Consume until we see OK or prompt
        serial.readString(lineBuf, '\n', 256, 1000);  // echo
        serial.readString(lineBuf, '\n', 256, 1000);  // response
        serial.readString(lineBuf, '>', 256, 1000);   // wait for prompt
        
        // Read 16KB in 4KB chunks (4 chunks per bank)
        for (int chunk = 0; chunk < 4; chunk++) {
            unsigned int addr = chunk * 0x1000;
            sprintf(cmdBuf, "readMulti %x 1000\r", addr);
            serial.writeString(cmdBuf);
            
            // Wait for "DATA:4096" marker
            serial.readString(lineBuf, '\n', 256, 2000);  // command echo
            serial.readString(lineBuf, '\n', 256, 2000);  // DATA:count line
            
            // Read 4096 bytes of raw data
            int bytesRead = serial.readBytes(buf, 4096, 30000, 100);
            if (bytesRead != 4096) {
                printf("\nError: Expected 4096 bytes, got %d at bank %d chunk %d\n", bytesRead, bank, chunk);
                outFile.close();
                return -1;
            }
            
            // Consume the OK and prompt
            serial.readString(lineBuf, '\n', 256, 1000);  // empty line after data
            serial.readString(lineBuf, '\n', 256, 1000);  // OK
            serial.readString(lineBuf, '>', 256, 1000);   // prompt
            
            outFile.write(buf, 4096);
            totalBytes += 4096;
        }
        
        printf("\rBank %d/%d (%lu KB)", bank + 1, numBanks, totalBytes / 1024);
        fflush(stdout);
    }
    
    outFile.close();
    printf("\nDone! Wrote %lu bytes to %s\n", totalBytes, filename.c_str());
    return 0;
}

int main(int argc, char** argv) {

    serialib serial;
    bool usingBankFiles = false;

    cxxopts::Options options("GTFO", "GameTank Flashing Overhauled");
    options.add_options()
        ("filenames", "The filename(s) to process", cxxopts::value<std::vector<string>>())
        ("p", "Serial port", cxxopts::value<string>()->default_value(SERIAL_PORT))
        ("r,read", "Read mode - dump flash contents to file")
        ("s,start", "Starting bank number for read (default 0)", cxxopts::value<int>()->default_value("0"))
        ("n,numbanks", "Number of banks to read (default 128 = 2MB)", cxxopts::value<int>()->default_value("128"));
    options.parse_positional({"filenames"});
    auto cmdLineResults = options.parse(argc, argv);
    char errorOpen = serial.openDevice(cmdLineResults["p"].as<string>().c_str(), 115200);

    if(cmdLineResults.count("filenames") < 1) {
        printf("need at least one filename");
        return 0;
    }

    auto filenames = cmdLineResults["filenames"].as<vector<string>>();

    if(errorOpen != 1) {
        printf("Couldn't connect to %s\n", cmdLineResults["p"].as<string>().c_str());
        return errorOpen;
    }

    printf("Connected to %s\n", cmdLineResults["p"].as<string>().c_str());

    // Read mode
    if(cmdLineResults.count("read")) {
        int startBank = cmdLineResults["start"].as<int>();
        int numBanks = cmdLineResults["numbanks"].as<int>();
        int result = readFlashToFile(serial, filenames.front(), startBank, numBanks);
        serial.closeDevice();
        return result;
    }

    // Write mode (original behavior)
    if(checkBankExtension(filenames.front().c_str()) != -1) {
        usingBankFiles = true;
    }  

    SerialStateMachine machine(serial);

    NopNode success(machine.endNode(), "SUCCESS");
    NopNode fail(machine.endNode(), "FAIL");
    success.msgTemplate.shouldWait = false;
    fail.msgTemplate.shouldWait = false;

    AbstractStateNode* shifter;

    //For full-chip flashing
    IncrementingNode incShifter(success, "shift %x\r", 12);
    incShifter.params[0].val = 0;
    incShifter.params[0].increment = 1;
    incShifter.count = 129;

    //For partial flashing
    StringListNode listShifter(success);

    StreamNode sendData(success, 4096);
    sendData.msgTemplate.skipLines = 0;

    if(usingBankFiles) {
        shifter = &listShifter;
        for(vector<string>::iterator fname = filenames.begin(); fname != filenames.end(); ++fname) {
            listShifter.cmds.push_back("shift " + getBankNum(fname->c_str()) + "\r");
            ifstream* stream = new ifstream();
            stream->open(fname->c_str(), ios::binary);
            sendData.streams.push_back(stream);
        } 
        listShifter.it = listShifter.cmds.begin();
    } else {
        ifstream* romFile = new ifstream();
        romFile->open(filenames.front().c_str(), ios::binary);
        sendData.streams.push_back(romFile);
        shifter = &incShifter;
    }

    shifter->msgTemplate.skipLines = 0;
    shifter->msgTemplate.slice = 5;
    sendData.it = sendData.streams.begin();

    IncrementingNode writeMulti(sendData, "writeMulti %x 1000\r", 32);
    writeMulti.msgTemplate.shouldWait = true;
    writeMulti.msgTemplate.skipLines = 0;
    writeMulti.params[0].val = 0;
    writeMulti.params[0].increment = 4096;
    writeMulti.count = 5;
    writeMulti.loop = true;

    writeMulti.loopLink = shifter;
    sendData.connect("ACK4096", &writeMulti);
    shifter->connect("> shi", &writeMulti);
    shifter->connect("shift", &writeMulti);

    SendStringNode eraseChip(fail, "eraseChip\r");
    eraseChip.msgTemplate.timeoutMs = 30000;
    eraseChip.msgTemplate.skipLines = 2;
    eraseChip.connect("Done", shifter);

    SendStringNode askVersion(fail, "version\r");
    askVersion.connect("GTCP2-0.0.2", &eraseChip);
    askVersion.msgTemplate.skipLines = 2;

    NopNode wakeupMsg(askVersion, "wakeupMsg");
    wakeupMsg.msgTemplate.delimiter = '!';

    machine.run(wakeupMsg);

    serial.closeDevice();

    return 0;
}
