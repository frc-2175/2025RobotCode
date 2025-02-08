class SourceIntake:
    # TODO: Wheel motor 1
    # TODO: Wheel motor 2

    # TODO: Make wheel motor 2 "follow" wheel motor 1

    def periodic(self):
        # TODO: Report encoder positions (and anything else) to NetworkTables
        pass

    def run_intake(self, speed: float):
        """
        Positive means moving coral from the funnel through to the arm. Negative is the reverse.
        Valid speeds range from -1 to 1.
        """
        # TODO: Actually make the motors go
        pass
