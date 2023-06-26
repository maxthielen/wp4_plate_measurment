from modules.ur5_nodes import UR5Node

if __name__ == '__main__':
    print("Init IO Node")
    ur5 = UR5Node()
    print("call set io")
    ur5.set_digital_output(1,1.0)

    # ur5.tigger_move.wait_for_
    # move_res = ur5.set_io()

    # ur5.trigger_start_position()
    # ur5.trigger_scan()

    input("Press Enter to continue...")

    # set_global()
    # main()
    # test_point_cloud_processing()

# import os
# from time import sleep
# from skimage import io

# from library.pcd_to_2d import spatial_res
# from modules.point_cloud import PointCloud

# from app.modules.flip_master_9000 import FlipMaster9000
# from app.modules.flippo import Flippo
# from app.modules.trispector_1060 import Trispector1060
# from app.modules.comand_line import CommandLinePublisher, CommandLineReceiver

# def set_global():
#     os.environ['img_path'] = "/data/img/"
#     os.environ['pcd_path'] = "/data/pcd/"
#     os.environ['prep_path'] = "/data/prep/"
#     os.environ['mm_per_dist'] = "1"

# def test_point_cloud_processing():
#     pc = PointCloud('03', '-bin.pcd')
#     pc.show()
#     features = pc.extract_features()
#     print(f"Features: {features}")

#     holes = pc.segment_image(io.imread(os.path.realpath('.') + os.getenv('prep_path') + '03-bin.png'), 1/spatial_res)
#     print(f"Holes: {holes}")

# def main():
#     rec = CommandLineReceiver()
#     pub = CommandLinePublisher()
    
#     move_it_node = UR5MoveNode()
#     io_node = UR5IONode()
#     tri = Trispector1060(move_it_node)
#     flip = Flippo(io_node)
    
#     FlipMaster9000(rec, pub, tri, flip)