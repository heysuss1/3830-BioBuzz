package org.firstinspires.ftc.teamcode.autos;
import static com.pedropathing.api.Paths.*;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
public class FlexAuto {
        private final PoseFactory poseFactory = PoseFactory.degrees();

        private final Pose start = poseFactory.of(55.9664, 5.9128, 90);
        private final Pose path1Start = poseFactory.of(55.9664, 5.9128, 180);
        private final Pose path1 = poseFactory.of(6.0478, 6.2968, 180);
        private final Pose point2Start = poseFactory.of(6.0478, 6.2968, 45);
        private final Pose point2 = poseFactory.of(48.2171, 115.7354, 90);
        private final Pose point2Control1 = poseFactory.of(7.3223, 93.4389, 0);
        private final Pose point3 = poseFactory.of(47.4077, 130.9586, 90);
        private final Pose point4 = poseFactory.of(3.9359, 117.8457, 270);

        public Path path1() {
            return line(path1Start, path1).linear(path1Start, path1);
        }

        public Path path2() {
            return curve(point2Start, point2Control1, point2).linear(point2Start, point2);
        }

        public Path path3() {
            return line(point2, point3).linear(point2, point3);
        }

        public Path path4() {
            return line(point3, point4).linear(point3, point4);
        }
}

