import asyncio
import os
from skimage import io
from library.pcd_to_2d import spatial_res
from modules.point_cloud import PointCloud
from modules.trispector_listener import TrispectorListener
from modules.ur5_nodes import UR5Node
from time import sleep

def test_point_cloud_processing():
    pc = PointCloud('02-bin', '.pcd')
    pc.isolate_plate(True)
    # pc.show()
    # features = pc.extract_features()
    # print(f"Features: {features}")

    # holes = pc.segment_image(io.imread(os.path.realpath('.') + os.getenv('prep_path') + '03-bin.png'), 1/spatial_res)
    # print(f"Holes: {holes}")

async def main():
    try:
        # test_point_cloud_processing()
        
        # ur5.set_digital_output(4,1.0)

        # sleep(1)

        # ur5.set_digital_output(5,1.0)

        # sleep(1)

        # ur5.set_digital_output(4,0.0)
        # ur5.set_digital_output(5,0.0)

        TrispectorListener()
        ur5 = UR5Node()

        while input("\nNext Plate? ...\n") != 'n':
            ur5.trigger_scan()

            if input("\nFlip? ...\n") == 'y':
                await ur5.trigger_flip()
                ur5.trigger_scan()

    except Exception as e:
        print(e)


if __name__ == '__main__':
    os.environ['mm_per_dist'] = "1"
    os.environ['img_path'] = "/data/png/"
    os.environ['pcd_path'] = "/data/pcd/"
   
    loop = asyncio.get_event_loop()
    loop.run_until_complete(main())
 
    input("Press Enter to shutdown...")


# def main():
#     rec = CommandLineReceiver()
#     pub = CommandLinePublisher()
    
#     move_it_node = UR5MoveNode()
#     io_node = UR5IONode()
#     tri = Trispector1060(move_it_node)
#     flip = Flippo(io_node)
    
#     FlipMaster9000(rec, pub, tri, flip)