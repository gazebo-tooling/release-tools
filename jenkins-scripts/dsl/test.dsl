import _configs_.*
import javaposse.jobdsl.dsl.Job

def ignition_ci_job = job("_test_dsl_jobs")
OSRFLinuxBase.create(ignition_ci_job)
