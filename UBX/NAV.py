"""Receiver Manager Messages: i.e. Satellite Status, RTC Status. """

from UBXMessage import initMessageClass, addGet
from Types import U1, U4, X1


@initMessageClass
class NAV:
    """Message class RXM."""

    _class = 0x01

    @addGet
    class STATUS:
        """§32.19.7.2 Broadcast Navigation Data Subframe."""

        _id = 0x03

        class Fields:
            iTOW = U4(0) # GPS time of week
            gpsFix = U1(4)
            flags = X1(5)
            fixStat = X1(6)
            flags2 = X1(7)
            ttff = U4(8) # time to first fix
            msss = U4(12)
