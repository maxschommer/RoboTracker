import numpy as np
import cv2
import apriltag
import collections
from argparse import ArgumentParser

from plt_3d import LivePlotter, Tag

# Taken from https://github.com/swatbotics/apriltag/blob/master/python/apriltag.py
def _camera_params(pstr):
    
    pstr = pstr.strip()
    
    if pstr[0] == '(' and pstr[-1] == ')':
        pstr = pstr[1:-1]

    params = tuple( [ float(param.strip()) for param in pstr.split(',') ] )

    assert( len(params) ==  4)

    return params


# Taken from https://github.com/swatbotics/apriltag/blob/master/python/apriltag.py
def _draw_pose(overlay, camera_params, tag_size, pose, z_sign=1):

    opoints = np.array([
        -1, -1, 0,
         1, -1, 0,
         1,  1, 0,
        -1,  1, 0,
        -1, -1, -2*z_sign,
         1, -1, -2*z_sign,
         1,  1, -2*z_sign,
        -1,  1, -2*z_sign,
    ]).reshape(-1, 1, 3) * 0.5*tag_size

    edges = np.array([
        0, 1,
        1, 2,
        2, 3,
        3, 0,
        0, 4,
        1, 5,
        2, 6,
        3, 7,
        4, 5,
        5, 6,
        6, 7,
        7, 4
    ]).reshape(-1, 2)
        
    fx, fy, cx, cy = camera_params

    K = np.array([fx, 0, cx, 0, fy, cy, 0, 0, 1]).reshape(3, 3)

    rvec, _ = cv2.Rodrigues(pose[:3,:3])
    tvec = pose[:3, 3]

    dcoeffs = np.zeros(5)

    ipoints, _ = cv2.projectPoints(opoints, rvec, tvec, K, dcoeffs)

    ipoints = np.round(ipoints).astype(int)

    ipoints = [tuple(pt) for pt in ipoints.reshape(-1, 2)]

    for i, j in edges:
        cv2.line(overlay, ipoints[i], ipoints[j], (0, 255, 0), 1, 16)


def main():

    parser = ArgumentParser(
        description='AprilTag based robot tracker')
    
    parser.add_argument('-t', '--tag_family', 
                        help="The tag family used for tracking, ex. 'tag16h5'.",
                        type=str
                        )
    parser.add_argument('-b', '--base_tag',
                        help="The tag number to define the base coordinate frame.",
                        type=int)
    parser.add_argument('-a', '--additional_tags',
                        help="Additional tags to track, whose coordinates will be defined with respect to the base frame.",
                        type=int,
                        nargs='+')
    parser.add_argument('-c', '--camera_params',
                        help="Intrinsic parameters for camera (in the form fx,fy,cx,cy)")
    parser.add_argument('-s', '--tag_size', type=float,
                        default=1.0,
                        help='Tag size in user-specified units (default=1.0)')

    args = parser.parse_args()
    cap = cv2.VideoCapture(0)

    options = apriltag.DetectorOptions(families=args.tag_family,
                                    border=1,
                                    nthreads=4,
                                    quad_decimate=1.0,
                                    quad_blur=0.0,
                                    refine_edges=True,
                                    refine_decode=False,
                                    refine_pose=False,
                                    debug=False,
                                    quad_contours=True,
                                    )
    camera_params = _camera_params(args.camera_params);

    detector = apriltag.Detector(options=options)

    plotter = LivePlotter(limits=[-5,5,-5,5,0,10])

    tag_ids = [args.base_tag, *(args.additional_tags)]
    tags = []
    for tag_id in tag_ids:
        tags.append(Tag(tag_id))
        plotter.add_artists(tags[-1].get_artists())


    while(True):
        # Capture frame-by-frame
        ret, frame = cap.read()

        # Our operations on the frame come here
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        detections = detector.detect(gray)

        for i, detection in enumerate(detections):
            if camera_params is not None:

                pose, e0, e1 = detector.detection_pose(detection,
                                                  camera_params,
                                                  1)

                if detection.tag_id in tag_ids:

                    for tag in tags:
                        if tag.tag_id == detection.tag_id:
                            tag.set_with_pose(camera_params, args.tag_size, pose)
                            if tag.tag_id in args.additional_tags:
                                R = tags[0].csys.get_rotation_matrix()
                                R_inv = np.transpose(R)
                                tag.csys.origin =  tag.csys.origin - tags[0].csys.origin
                                tag.csys.rotate(R_inv)
                                tag.csys.origin = np.dot(R_inv, tag.csys.origin)

                            _draw_pose(gray,
                                       camera_params,
                                       args.tag_size,
                                       pose)

                            print("Tag ID: ", detection.tag_id)
                            print()

        R = tags[0].csys.get_rotation_matrix()
        R_inv = np.transpose(R)
        tags[0].csys.origin =  tags[0].csys.origin - tags[0].csys.origin
        tags[0].csys.rotate(R_inv)
        tags[0].csys.origin = np.dot(tags[0].csys.origin, R_inv)

        plotter.clear_artists()
        for tag in tags:
            plotter.add_artists(tag.get_artists(arrow_len=2))
        plotter.update_plt()
        # print(detections)
        # Display the resulting frame
        cv2.imshow('frame',gray)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    # When everything done, release the capture
    cap.release()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()




