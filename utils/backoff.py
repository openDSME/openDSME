#!/usr/bin/env python3

def main():
    SO = 3
    MO = 5
    print("SO:", SO)
    print("MO:", MO)

    symbolsPerSlot = pow(2, SO) * 60
    symbolsPerSuperFrame = symbolsPerSlot * 16
    symbolsPerCap = symbolsPerSlot * 8
    print("symbolsPerSlot:", symbolsPerSlot)
    print("symbolsPerSuperFrame:", symbolsPerSuperFrame)
    print("symbolsPerCap:", symbolsPerCap)

    now                         = 2390534
    symbolsSinceCapFrameStart   = 44278
    totalWaitTime               = 123040
    usableCapPhaseEnd           = 17088
    print("now:", now)
    print("symbolsSinceCapFrameStart:", symbolsSinceCapFrameStart)
    print("totalWaitTime:", totalWaitTime)

    packetLength = symbolsPerSlot + symbolsPerCap - usableCapPhaseEnd
    print("packetLength:", packetLength)

    capFrameStart = now - symbolsSinceCapFrameStart
    print("capFrameStart:", capFrameStart)

    timerEndTime = capFrameStart + symbolsPerSlot + totalWaitTime
    print("timerEndTime:", timerEndTime)


    for i in range(0, pow(2, MO - SO) + 1):
        print(i % pow(2, MO - SO), "start", capFrameStart + symbolsPerSuperFrame * i, "\tcap:", capFrameStart + symbolsPerSuperFrame * i + symbolsPerSlot, "\tcapEnd", capFrameStart + symbolsPerSuperFrame * i + symbolsPerSlot + symbolsPerCap - packetLength)


if __name__ == "__main__":
    main()
