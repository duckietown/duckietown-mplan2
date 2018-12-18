__all__ = [
    'WorkerBase',
]


import abc
import rospy


class WorkerBase(object):
    """
    Generic worker object which should be instantiated in a node. It should
    be designe such that only one worker per node is needed.

    If in standalone mode the node will automatically call the advance function
    with the given frequency, else the user has to be concerned to call the
    advance method when desired.
    """
    __metaclass__ = abc.ABCMeta

    def __init__(self, standalone=True, frequency=-1):
        """
        Instantiate all base member objects and call init methods. If in
        standalone mode the main loop will be entered directly afterwords.

        Parameters
        ----------
        standalone : bool, optional
            Whether the worker is running standalone or not.
        frequency : int, optional
            The frequency of the advance method if called in standalone mode.

        Returns
        -------
        none
        """

        self.frequency = frequency
        self.standalone = standalone
        self.init()
        self.initIO()

        # if in standalone mode worker automatically starts its main loop
        if self.standalone:
            self.startup()

    def __del__(self):
        """
        Cleanup members.

        Parameters
        ----------
        none

        Returns
        -------
        none
        """
        self.shutdown()

    @abc.abstractmethod
    def init(self):
        """
        Initialise all members of class. To be implemented in derived class.

        Parameters
        ----------
        none

        Returns
        -------
        none
        """
        pass

    @abc.abstractmethod
    def initIO(self):
        """
        Instantiate all input / output behaviour of worker. This mainly
        includes ros-publishers / -subscribers and advertised services.
        To be implemented in derived class.

        Parameters
        ----------
        none

        Returns
        -------
        none
        """
        pass

    def startup(self):
        """
        Do any preparation needed before starting main loop. Normally should not
        be changed.

        Parameters
        ----------
        none

        Returns
        -------
        none
        """
        self.mainLoop()

    def mainLoop(self):
        """
        The main loop running at the specified frequency. Is only called if in
        standalone mode.

        Parameters
        ----------
        none

        Returns
        -------
        none
        """
        assert self.frequency >= -1
        last = rospy.Time.now()

        # if frequency = -1 run at max speed
        if self.frequency == -1:
            while not rospy.is_shutdown():
                now = rospy.Time.now()
                Ts = (now - last).to_sec()
                last = now
                self.advance(Ts)
        else:
            r = rospy.Rate(self.frequency)
            while not rospy.is_shutdown():
                now = rospy.Time.now()
                Ts = (now - last).to_sec()
                last = now
                self.advance(Ts)
                r.sleep()

    @abc.abstractmethod
    def advance(self, Ts=1.0):
        """
        Main method where data is processed and output generated. Gets called
        once in every loop with frequency self.frequency. To be implemented in
        derived class.

        Parameters
        ----------
        Ts : double, optional
            The time in seconds since the advance method was executed the last time.

        Returns
        -------
        none
        """
        pass

    @abc.abstractmethod
    def shutdown(self):
        """
        Clean up class before process end. To be implemented in derived class.

        Parameters
        ----------
        none

        Returns
        -------
        none
        """
        pass
