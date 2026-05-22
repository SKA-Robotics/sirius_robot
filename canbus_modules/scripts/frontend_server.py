#!/usr/bin/env python3
import os
import sys
import threading
import ssl
from pathlib import Path
from typing import Optional

import rospy
import rospkg

from flask import Flask, Response, send_from_directory, jsonify
import tornado.wsgi
import tornado.httpserver
import tornado.ioloop

from .certificate_manager import generate_server_cert, cleanup_certificate_files


def find_frontend_dist() -> Optional[Path]:
    # Try installed package layout (catkin installed share)
    try:
        rp = rospkg.RosPack()
        package_path = rp.get_path("web_robot_control")
        installed_path = Path(package_path) / "frontend" / "dist"
        if installed_path.exists():
            return installed_path
    except Exception:
        pass

    # Dev layout (package source tree)
    dev_path = Path(__file__).parent.parent / "frontend" / "dist"
    if dev_path.exists():
        return dev_path

    return None


def get_package_path(package_name: str) -> Optional[str]:
    try:
        rp = rospkg.RosPack()
        return rp.get_path(package_name)
    except Exception:
        return None


def create_app(config_str: str, frontend_dist_path: Path) -> Flask:
    app = Flask(__name__)

    @app.route("/models/packages/<package_name>/<path:file_path>")
    def serve_package_file(package_name: str, file_path: str):
        package_path = get_package_path(package_name)

        if not package_path:
            rospy.logwarn(f"Package {package_name} not found")
            return jsonify({"error": f"Package {package_name} not found"}), 404

        full_file_path = os.path.join(package_path, file_path)

        if not os.path.abspath(full_file_path).startswith(
                os.path.abspath(package_path)):
            rospy.logwarn(
                f"Access denied for {file_path} in package {package_name}")
            return jsonify({"error": "Access denied"}), 403

        if os.path.exists(full_file_path) and os.path.isfile(full_file_path):
            try:
                rospy.logdebug(
                    f"Serving file: {file_path} from package {package_name}")
                return send_from_directory(os.path.dirname(full_file_path),
                                           os.path.basename(full_file_path))
            except Exception as e:
                rospy.logerr(f"Error serving file {file_path}: {str(e)}")
                return jsonify({"error": f"Error serving file: {str(e)}"}), 500
        else:
            rospy.logwarn(
                f"File {file_path} not found in package {package_name}")
            return (
                jsonify({
                    "error":
                    f"File {file_path} not found in package {package_name}"
                }),
                404,
            )

    @app.route("/")
    def serve_frontend():
        return send_from_directory(frontend_dist_path, "index.html")

    @app.route("/config.yaml")
    def serve_config():
        return Response(config_str, mimetype="text/yaml")

    @app.route("/<path:path>")
    def serve_static(path: str):
        file_path = frontend_dist_path / path
        if file_path.exists() and file_path.is_file():
            return send_from_directory(frontend_dist_path, path)
        else:
            return send_from_directory(frontend_dist_path, "index.html")

    return app


def run_ros_node():
    rospy.loginfo("ROS spin thread started")
    try:
        rospy.spin()
    except Exception as e:
        rospy.logerr(f"Error in ROS spin thread: {e}")
    rospy.loginfo("ROS spin thread finished")


def main() -> None:
    rospy.init_node("web_robot_control_server", anonymous=False)

    port = rospy.get_param("~port", 8080)
    host = rospy.get_param("~host", "0.0.0.0")
    config = rospy.get_param("~config", "")

    frontend_dist_path = find_frontend_dist()

    if not frontend_dist_path:
        rospy.logerr("Frontend dist directory not found.")
        rospy.logerr(
            "Please ensure the frontend is built and the package is properly installed."
        )
        rospy.signal_shutdown("frontend not found")
        sys.exit(1)

    app = create_app(config, frontend_dist_path)

    rospy.loginfo(f"Starting Web Robot Control Server on {host}:{port}")
    rospy.loginfo(f"Frontend served from: {frontend_dist_path}")

    try:
        ros_thread = threading.Thread(target=run_ros_node, daemon=True)
        ros_thread.start()

        server_cert_file, server_key_file = generate_server_cert()

        if server_cert_file and server_key_file:
            ssl_context = ssl.create_default_context(ssl.Purpose.CLIENT_AUTH)
            ssl_context.load_cert_chain(server_cert_file, server_key_file)

            container = tornado.wsgi.WSGIContainer(app)
            http_server = tornado.httpserver.HTTPServer(
                container, ssl_options=ssl_context)
            http_server.listen(port, address=host)
            rospy.loginfo("Tornado HTTPS server started")
            tornado.ioloop.IOLoop.current().start()
        else:
            rospy.logerr("SSL setup failed, closing server")

    except rospy.ROSInterruptException:
        rospy.loginfo("ROSInterruptException, shutting down...")
    except KeyboardInterrupt:
        rospy.loginfo("Received KeyboardInterrupt, shutting down...")
    except SystemExit:
        rospy.loginfo("Received SystemExit, shutting down...")
    except Exception as e:
        rospy.logerr(f"Unexpected error: {e}")

    rospy.loginfo("Stopping Tornado IOLoop...")
    try:
        tornado.ioloop.IOLoop.current().stop()
    except Exception:
        pass

    rospy.loginfo("Shutting down ROS...")
    rospy.signal_shutdown("server shutdown")

    cleanup_certificate_files(server_cert_file, server_key_file)


if __name__ == "__main__":
    main()
