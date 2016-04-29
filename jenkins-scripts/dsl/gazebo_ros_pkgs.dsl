import _configs_.*
import javaposse.jobdsl.dsl.Job

def ros_distros    = Globals.get_ros_suported_distros()
def ci_arch        = 'amd64'
def ci_gpu         = 'nvidia'

ros_distros.each { ros_distro ->
  ubuntu_distros = Globals.gpu_by_distro[ros_distro]
 
  ubuntu_distros.each { ubuntu_distro ->
    def ci_job = job("ros_gazebo_pkgs-ci-pr_any_${ros_distro}-${ubuntu_distro}-${ci_arch}-gpu-${ci_gpu}")
    OSRFLinuxCompilation.create(ci_job)
}
