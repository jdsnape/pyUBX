"""Receiver Manager Messages: i.e. Satellite Status, RTC Status. """

from UBXMessage import initMessageClass, addGet
from Types import U1, U4, R8, R4, X1, U2, I1


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
                dpMeas = R4(3)
                gnssId = U1(4)
                svId = U1(5)
                reserved2 = U1(6)
                freqId = U1(7)
                lockTime = U2(8)
                cn0 = U1(9)
                prStdev = X1(10)
                cpStdev = X1(11)
                doStdev = X1(12)
                trkStat = X1(13)
                reserved3 = U1(14)
