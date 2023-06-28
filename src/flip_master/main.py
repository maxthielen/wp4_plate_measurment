import os
from skimage import io
from library.pcd_to_2d import spatial_res
from modules.point_cloud import PointCloud
# from modules.trispector_listener import TrispectorListener
# from modules.ur5_nodes import UR5Node

def test_point_cloud_processing():
    pc = PointCloud('01-bin', '.pcd')
    pc.show()
    features = pc.extract_features()
    print(f"Features: {features}")

    holes = pc.segment_image(io.imread(os.path.realpath('.') + os.getenv('prep_path') + '03-bin.png'), 1/spatial_res)
    print(f"Holes: {holes}")

def main():
    try:
        test_point_cloud_processing()
        # TrispectorListener()

        # ur5 = UR5Node()
        # ur5.trigger_scan()

        # ur5.set_digital_output(0,1.0)
        # ur5.set_digital_output(5,0.0)
    except Exception as e:
        print(e)


if __name__ == '__main__':
    os.environ['mm_per_dist'] = "1"
    os.environ['img_path'] = "/data/png/"
    os.environ['pcd_path'] = "/data/pcd/"

    main()

    input("Press Enter to continue...")


# def main():
#     rec = CommandLineReceiver()
#     pub = CommandLinePublisher()
    
#     move_it_node = UR5MoveNode()
#     io_node = UR5IONode()
#     tri = Trispector1060(move_it_node)
#     flip = Flippo(io_node)
    
#     FlipMaster9000(rec, pub, tri, flip)