import json


def main():
    with open("testpath1.traj", "r", encoding="UTF-8") as fp:
        traj = json.load(fp)
    print("vec![")
    for sample in traj["trajectory"]["samples"]:
        t, x, y, heading, vx, vy, omega = (
            sample[key] for key in ["t", "x", "y", "heading", "vx", "vy", "omega"]
        )
        print(
            f"  AutonSetpoint{{t: {t}, x: {x}, y: {y}, heading: {heading}, vx: {vx}, vy: {vy}, omega: {omega}}},"
        )
    print("]")


if __name__ == "__main__":
    main()
