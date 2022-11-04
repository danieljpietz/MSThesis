class Platoon(pycbf2.NESystem):
    def __init__(self, length, t_safety, v_goal):
        super(Platoon, self).__init__()

        for i in range(length):
            vehicle = Link(
                parent=self,
                mass=1,
                center_of_mass=[0, 0, 0],
                inertia_tensor=np.eye(3),
                index=i,
                axis=[1, 0, 0],
                link_type=LinkType.prismatic,
                rotation_local=np.eye(3),
                position=[0, 0, 0],
            )

        t, x, xdot = self.cbf_vars()

        class Controller(cbf.ControlFunc):
            def __init__(self):
                self.cbf = 1
                for i in range(length - 1):
                    self.cbf *= (x[i + 1] - x[i]) - t_safety * xdot[i]
                self.t = 0

            def input_matrix(self, x, xdot):
                return np.eye(length)

            def uref(self, x, xdot):
                return v_goal - xdot

        self.controller = Controller()