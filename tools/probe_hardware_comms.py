#!/usr/bin/env python3
"""Finite BRAKE-mode motor and receive-only IMU communication diagnostics.

All motor packets use mode 0 (SDK BRAKE), tau=kp=kd=q=dq=0. This does not run
calibration, FOC, standing, or the robot controller. BRAKE mode is a protocol
state, not proof that measured physical torque is zero. Do not run beside any
other process using these ports. Output includes SDK and independent CRC checks.
"""
import argparse,datetime,json,pathlib,platform,shutil,subprocess,sys

def main():
    root=pathlib.Path(__file__).resolve().parents[1]
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--sdk',type=pathlib.Path,default=pathlib.Path('/home/orangepi/unitree_actuator_sdk-main/unitree_actuator_sdk-main'))
    parser.add_argument('--output',type=pathlib.Path,default=root/'log'/('hardware-comms-'+datetime.datetime.now().strftime('%Y%m%d-%H%M%S')))
    parser.add_argument('--samples',type=int,default=20)
    parser.add_argument('--skip-motors',action='store_true')
    parser.add_argument('--skip-imu',action='store_true')
    args=parser.parse_args()
    if not 1<=args.samples<=500:parser.error('samples must be 1..500')
    if args.skip_motors and args.skip_imu:parser.error('at least one probe must be enabled')
    ports=[] if args.skip_motors else ['/dev/ttyS3','/dev/ttyS4','/dev/ttyS7','/dev/ttyS8']
    if not args.skip_imu:ports.append('/dev/ttyUSB0')
    if not shutil.which('fuser'):raise RuntimeError('fuser is required to check serial ownership')
    for port in ports:
        if not pathlib.Path(port).exists():raise RuntimeError('serial port missing: '+port)
    check=subprocess.run(['fuser',*ports],capture_output=True,text=True)
    if check.returncode!=1 or check.stdout.strip() or check.stderr.strip():
        raise RuntimeError('serial ports are occupied or ownership check failed: '+check.stdout+check.stderr)
    args.output.mkdir(parents=True,exist_ok=True)
    result={'timestamp_utc':datetime.datetime.now(datetime.timezone.utc).isoformat(),'mode':0,'tau':0,'kp':0,'kd':0,'ports':ports,'output':str(args.output)}
    if not args.skip_motors:
        if platform.machine() not in ('aarch64','arm64'):raise RuntimeError('this probe requires ARM64 Unitree SDK')
        binary=args.output/'probe_motor_comms'
        compile_cmd=['c++','-std=c++17','-O2',str(root/'tools/probe_motor_comms.cpp'),'-I',str(args.sdk/'include'),'-L',str(args.sdk/'lib'),'-Wl,-rpath,'+str(args.sdk/'lib'),'-lUnitreeMotorSDK_Arm64','-pthread','-o',str(binary)]
        subprocess.run(compile_cmd,check=True)
        with (args.output/'motors.log').open('w') as log:
            run=subprocess.run([str(binary),str(args.output/'motors.json'),str(args.samples)],stdout=log,stderr=subprocess.STDOUT,timeout=120)
        result['motor_exit_code']=run.returncode
        if (args.output/'motors.json').exists():result['motors']=json.loads((args.output/'motors.json').read_text())
    if not args.skip_imu:
        with (args.output/'imu.log').open('w') as log:
            run=subprocess.run([sys.executable,str(root/'tools/probe_imu_comms.py'),'--output',str(args.output/'imu.json')],stdout=log,stderr=subprocess.STDOUT,timeout=15)
        result['imu_exit_code']=run.returncode
        if (args.output/'imu.json').exists():result['imu']=json.loads((args.output/'imu.json').read_text())
    (args.output/'summary.json').write_text(json.dumps(result,indent=2)+'\n')
    print('Communication evidence:',args.output/'summary.json')
    if 'motors' in result:
        for motor in result['motors']['motors']:print(motor['leg'],'ID',motor['id'],str(motor['valid'])+'/'+str(args.samples),'valid, error',motor['last_error'])
    if 'imu' in result:print('IMU rates:',result['imu'].get('rate_hz',{}),'CRC/errors:',result['imu'].get('errors',{}))
    return 2 if result.get('motor_exit_code',0) or result.get('imu_exit_code',0) else 0
if __name__=='__main__':
    try:sys.exit(main())
    except (RuntimeError,OSError,subprocess.SubprocessError) as error:print('Probe stopped:',error,file=sys.stderr);sys.exit(1)
