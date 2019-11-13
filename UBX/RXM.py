"""Receiver Manager Messages: i.e. Satellite Status, RTC Status. """

from UBXMessage import initMessageClass, addGet
from Types import U1, U4, R8, R4, X1, U2, I1, I4


@initMessageClass
class RXM:
    """Message class RXM."""

    _class = 0x02

    @addGet
    class SFRBX:
        """§32.19.7.2 Broadcast Navigation Data Subframe."""

        _id = 0x13

        class Fields:
            gnssId = U1(1)
            svId = U1(2)
            reserved1 = U1(3)
            freqId = U1(4)
            numWords = U1(5)
            chn = U1(6)
            ver = U1(7)
            reserved2 = U1(8)

            class Repeated:
                dwrd = U4(1)

    @addGet
    class RAWX:
        """§32.19.4.1 Multi-GNSS Raw Measurement Data."""

        _id = 0x15

        class Fields:
            rcvrTow = R8(1)
            week = U2(2)
            leapS = I1(3)
            numMeas = U1(4)
            recStat = X1(5)
            version = U1(6)
            reserved1_1 = U1(7)
            reserved1_2 = U1(8)

            class Repeated:
                prMeas = R8(1)
                cpMeas = R8(2)
                doMes = R4(3)
                gnssId = U1(4)
                svId = U1(5)
                reserved2 = U1(6)
                freqId = U1(7)
                lockTime = U2(8)
                cno = U1(9)
                prStdev = X1(10)
                cpStdev = X1(11)
                doStdev = X1(12)
                trkStat = X1(13)
                reserved3 = U1(14)

    @addGet
    class MEASX:
        """§32.18.2.1 Satellite Measurements for RRLP"""

        _id = 0x14

        class Fields:
            version = U1(1)
            reserved1_1 = U1(2)
            reserved1_2 = U1(3)
            reserved1_3 = U1(4)
            gpsTOW = U4(5)
            gloTOW = U4(6)
            bdsTOW = U4(7)
            reserved2_1 = U1(8)
            reserved2_2 = U1(9)
            reserved2_3 = U1(10)
            reserved2_4 = U1(11)
            qzssTOW = U4(12)
            gpsTOWacc = U2(13)
            gloTOWacc = U2(14)
            bdsTOWacc = U2(15)
            reserved3_1 = U1(16)
            reserved3_2 = U1(17)
            qzaaTOWacc = U2(18)
            numSV = U1(19)
            flags = U1(20)
            reserved4_1 = U1(21)
            reserved4_2 = U1(22)
            reserved4_3 = U1(23)
            reserved4_4 = U1(24)
            reserved4_5 = U1(25)
            reserved4_6 = U1(26)
            reserved4_7 = U1(27)
            reserved4_8 = U1(28)

            class Repeated:
                gnssId = U1(29)
                svId = U1(30)
                cNo = U1(31)
                mpathIndic = U1(32)
                dopplerMS = I4(33)
                dopplerHz = I4(34)
                wholeChips = U2(35)
                fracChips = U2(36)
                codePhase = U4(37)
                intCodePhase = U1(38)
                pseuRangeRMSErr = U1(39)
                reserved5_1 = U1(40)
                reserved5_2 = U1(41)
