"""

Trajectory generation from setpoints

author: Ashwin Bose (@atb033)

"""
import argparse
import yaml
import numpy as np

class TrajectoryGenerator(object):
    def __init__(self, setpoint_dict):
        self.setpoint_dict = setpoint_dict
        # print(self.setpoint_dict)
        self.generate_list_from_dict()

    def generate_list_from_dict(self):
        self.setpoint_list = {}

        for agent, schedule in self.setpoint_dict.items():
            self.setpoint_list[agent] = []
            for step in schedule:
                self.setpoint_list[agent].append([step['t'], step['x'], step['y']])
            self.setpoint_list[agent] = sorted(self.setpoint_list[agent])

    def get_point_vel(self, t, agent):

        schedule = self.setpoint_list[agent]
        for i, step in enumerate(schedule):
            if t < step[0]:
                point_a = np.array(schedule[i-1][1:])
                t_a = schedule[i-1][0]
            
                point_b = np.array(schedule[i][1:])
                t_b = schedule[i][0]
                break

        point_mid = point_a + (point_b - point_a) * t / (t_b - t_a)
        # print(point_mid)
        dist = np.linalg.norm(point_b-point_a)
        v = dist/(t_b-t_a)

        return (point_mid[0], point_mid[1], v)
        
def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("schedule", help="input file containing map and obstacles")
    args = parser.parse_args()
    
    # Read from input file
    with open(args.schedule, 'r') as schedule_file:
        try:
            schedule = yaml.load(schedule_file, Loader=yaml.FullLoader)
        except yaml.YAMLError as exc:
            print(exc)

    t_gen = TrajectoryGenerator(schedule)
    print(t_gen.get_point_vel(2.5, 'agent0'))



if __name__ == "__main__":
    main()
