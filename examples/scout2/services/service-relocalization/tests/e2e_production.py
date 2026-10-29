"""One isolated production-entry regression: Atlas, native matching and ROS handoff."""
import json
import os
from pathlib import Path
import sys
import threading
import time
import unittest
from concurrent.futures import ThreadPoolExecutor
import grpc
import numpy as np
import rclpy
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import PoseWithCovarianceStamped, TransformStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2, PointField
from tf2_ros import StaticTransformBroadcaster, TransformBroadcaster
import atlas_pb2
import map_pb2
import robonix_contracts_pb2_grpc as contracts
from e2e_atlas import AtlasIntegrationTests, wait_for
from backend_matching import MatchingTests
from relocalization_rbnx.coordination import MapWriter
from relocalization_rbnx.navigation import check

class FullCheck(AtlasIntegrationTests):
    def command(self, command):
        """Send the configured production profile through the real Driver."""
        import lifecycle_pb2
        return self.driver(lifecycle_pb2.Driver_Request(command=command,
            config_json=json.dumps(self.config)),timeout=15)

    def test_full_entry(self):
        """Verify automatic startup, live geometry, reload recovery and deactivation."""
        snapshot = MatchingTests()
        snapshot.setUp()
        self.addCleanup(snapshot.doCleanups)
        index = snapshot.root/'index-0'
        root = Path(self.temp.name)
        profile = root/'profile.json'
        profile.write_text(json.dumps(dict(index=str(index),capture=dict(scan_duration_ns=40_000_000,
            header_is_end=True,max_age_s=1.0),base_from_sensor=np.eye(4).reshape(-1).tolist(),
            registration=dict(base_z=0,roll=0,pitch=0,max_rmse_m=.04),
            search=dict(yaw_step=float(np.pi/2),min_score=.98,max_candidates=10),
            matching=dict(max_blocked_ray_fraction=.2))))
        self.config = dict(map_id='synthetic',mapping_provider_id='mapping_e2e',lidar_provider_id='lidar_e2e',
            odom_provider_id='odom_e2e',maps_dir=str(root/'maps'),index_dir=str(snapshot.root),
            profile_file=str(profile),coordination_dir=str(root/'coord'),timeout_s=5,status_period_s=.1,
            auto_relocalize=True)
        self.dependency('lidar_e2e','robonix/primitive/lidar','lidar3d')
        self.dependency('odom_e2e','robonix/primitive/chassis','odom')
        self.dependency('mapping_e2e','robonix/service/map','lifecycle')
        self.dependency('mapping_e2e','robonix/service/map','pose')
        calls = []
        class Mapping(contracts.RobonixServiceMapPoseEstimateServicer):
            def PoseEstimate(self, req, ctx):
                calls.append(req)
                return map_pb2.PoseEstimate_Response(ok=True)
        server = grpc.server(ThreadPoolExecutor(max_workers=1))
        contracts.add_RobonixServiceMapPoseEstimateServicer_to_server(Mapping(),server)
        port = server.add_insecure_port('127.0.0.1:0'); server.start()
        self.addCleanup(lambda:server.stop(0).wait())
        self.atlas.DeclareCapability(atlas_pb2.DeclareCapabilityRequest(provider_id='mapping_e2e',
            contract_id='robonix/service/map/pose_estimate',transport=atlas_pb2.TRANSPORT_GRPC,
            endpoint=f'127.0.0.1:{port}',params=atlas_pb2.TransportParams(grpc=atlas_pb2.GrpcParams(
                service_name='RobonixServiceMapPoseEstimate',method='PoseEstimate'))),timeout=3)
        old = dict(os.environ)
        os.environ.update(self.env)
        self.addCleanup(lambda: (os.environ.clear(),os.environ.update(old)))
        context = Context(); rclpy.init(context=context)
        node = rclpy.create_node('full_chain_inputs',context=context)
        executor = SingleThreadedExecutor(context=context); executor.add_node(node)
        thread = threading.Thread(target=executor.spin,daemon=True); thread.start()
        def close_ros():
            """Stop owned executor resources before the fixture context."""
            executor.shutdown(); thread.join(3); node.destroy_node(); context.shutdown()
        self.addCleanup(close_ros)
        cloud_pub = node.create_publisher(PointCloud2,'/e2e/lidar_e2e/lidar3d',qos_profile_sensor_data)
        odom_pub = node.create_publisher(Odometry,'/e2e/odom_e2e/odom',qos_profile_sensor_data)
        pose_pub = node.create_publisher(PoseWithCovarianceStamped,'/e2e/mapping_e2e/pose',1)
        static = StaticTransformBroadcaster(node); dynamic = TransformBroadcaster(node)
        transform = TransformStamped(); transform.header.frame_id='base_link'; transform.child_frame_id='laser'
        transform.header.stamp=node.get_clock().now().to_msg(); transform.transform.rotation.w=1.0
        static.sendTransform(transform)
        def odom():
            """Publish stationary odometry and matching timestamped map feedback."""
            stamp = node.get_clock().now().to_msg()
            message = Odometry(); message.header.stamp=stamp; message.header.frame_id='odom'
            message.child_frame_id='base_link'; message.pose.pose.orientation.w=1.0
            odom_pub.publish(message)
            tf = TransformStamped(); tf.header.stamp=stamp; tf.header.frame_id='odom'; tf.child_frame_id='base_link'
            tf.transform.rotation.w=1.0; dynamic.sendTransform(tf)
            p = PoseWithCovarianceStamped(); p.header.stamp=stamp; p.header.frame_id='map'
            p.pose.pose.orientation.w=1.0; p.pose.pose.position.x=float(snapshot.pose[0]);p.pose.pose.position.y=float(snapshot.pose[1]);pose_pub.publish(p)
        def cloud():
            """Publish independent synthetic scans on the declared lidar endpoint."""
            c = PointCloud2(); c.header.stamp=node.get_clock().now().to_msg();c.header.frame_id='laser'
            c.height=1;c.width=len(snapshot.scan);c.point_step=12;c.row_step=12*c.width;c.is_dense=True
            c.fields=[PointField(name=name,offset=i*4,datatype=7,count=1) for i,name in enumerate(('x','y','z'))]
            c.data=snapshot.scan.astype('<f4').tobytes();cloud_pub.publish(c)
        node.create_timer(.02,odom);node.create_timer(.12,cloud)
        writer=MapWriter(root/'coord','mapping_e2e');self.addCleanup(writer.close)
        writer.mapping('first_map')
        self.assertTrue(self.command(0).ok)
        result=self.command(1);self.assertTrue(result.ok,result.error)
        import relocalization_pb2 as pb
        status=lambda:self.status(pb.GetStatus_Request(),timeout=3).status
        wait_for(lambda:'waiting for a localization map' in status().detail,timeout=3)
        time.sleep(1.2)
        self.assertFalse(status().busy)
        self.assertFalse(status().localization_valid)
        self.assertEqual(status().request_id, '')
        self.assertEqual(calls, [])
        self.assertFalse((root/'maps').exists())
        writer.loaded(dict(map_id='synthetic',generation=1,database_sha256='a'*64))
        wait_for(lambda:status().localization_valid,timeout=8)
        self.assertEqual(len(calls),1)
        wait_for(lambda:check(root/'coord','relocalization_e2e')[0],timeout=2)
        first=status().request_id
        writer.invalidate()
        self.assertFalse(check(root/'coord','relocalization_e2e')[0])
        wait_for(lambda:not status().localization_valid,timeout=2)
        writer.loaded(dict(map_id='synthetic',generation=1,database_sha256='a'*64))
        wait_for(lambda:status().localization_valid,timeout=8)
        self.assertNotEqual(status().request_id,first)
        self.assertEqual(len(calls),2)
        self.assertTrue(self.command(2).ok)
        self.assertFalse(status().active)
        self.config['auto_relocalize'] = False
        self.assertTrue(self.command(0).ok)
        self.assertTrue(self.command(1).ok)
        time.sleep(1.5)
        self.assertEqual(status().request_id, '')
        self.assertFalse(status().busy)
        self.assertFalse(status().localization_valid)
        self.assertEqual(len(calls), 2)
        request = contracts.RobonixServiceRelocalizationRelocalizeByMapStub(
            self.channel(self.connect('relocalize_by_map').endpoint)).RelocalizeByMap
        response = request(pb.RelocalizeByMap_Request(map_id='synthetic'), timeout=3)
        self.assertTrue(response.accepted, response.detail)
        wait_for(lambda:status().localization_valid,timeout=8)
        self.assertEqual(status().request_id, response.request_id)
        self.assertEqual(len(calls), 3)
        self.assertTrue(self.command(2).ok)
        self.assertFalse(status().active)
        self.assertTrue(self.command(3).ok)

if __name__=='__main__':
    result=unittest.TextTestRunner(verbosity=2).run(unittest.TestSuite([FullCheck('test_full_entry')]))
    sys.exit(not result.wasSuccessful())
