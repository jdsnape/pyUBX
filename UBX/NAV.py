"""Receiver Manager Messages: i.e. Satellite Status, RTC Status. """

from UBXMessage import initMessageClass, addGet
from Types import U1, U2, I4, U4, X1, X4, I2, I1


@initMessageClass
class NAV:
    """Message class NAV"""

    _class = 0x01

    @addGet
    class STATUS:
        """§32.19.7.2 Broadcast Navigation Data Subframe."""

        _id = 0x03

        class Fields:
            iTOW = U4(0)  # GPS time of week
            gpsFix = U1(4)
            flags = X1(5)
            fixStat = X1(6)
            flags2 = X1(7)
            ttff = U4(8)  # time to first fix
            msss = U4(12)

    @addGet
    class SAT:
        """§32.17.17.1 Satellite information."""

        _id = 0x35

        class Fields:
            iTOW = U4(1)
            version = U1(2)
            numSvs = U1(3)
            reserved1_1 = U1(4)
            reserved1_2 = U1(5)

            class Repeated:
                gnssId = U1(6)
                svId = U1(7)
                cno = U1(8)
                elev = I1(9)
                azim = I2(10)
                prRes = I2(11)
                flags = X4(12)

    @addGet
    class PVT:
        """§32.17.14.1 Position Velocity Time solution."""

        _id = 0x07

        class Fields:
            iTOW = U4(1)
            year = U2(2)
            month = U1(3)
            day = U1(4)
            hour = U1(5)
            minute = U1(5)
            sec = U1(5)
            valid = X1(6)
            tAcc = U4(7)
            nano = I4(8)
            fixType = U1(9)
            flags = X1(10)
            flags2 = X1(11)
            numSV = U1(12)
            lon = I4(13)
            lat = I4(14)
            height = I4(15)
            hMSL = I4(16)
            hAcc = U4(17)
            vAcc = U4(18)
            velN = I4(19)
            velE = I4(20)
            velD = I4(21)
            gSpeed = I4(22)
            headMot = I4(23)
            sAcc = U4(24)
            headAcc = U4(25)
            pDOP = U2(26)
            reserved1_1 = U4(27)
            reserved1_2 = U2(28)
            headVeh = I4(29)
            magDec = I2(30)
            magAcc = U2(31)
