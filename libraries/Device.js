
class Device{

    deviceLink; // = new Board();

	deviceID;
	mechanism;
	
	communicationType;
	
	actuatorsActive    = 0;
	motors             = new Actuator();//[0];
	
	encodersActive     = 0;
	encoders           = new Sensor();//[0];
	
	sensorsActive      = 0;
	sensors            = new Sensor();//();//[0];
	
	pwmsActive		     = 0;
  pwms 			         = new Pwm();//[0];
    
    actuatorPositions  = [0, 0, 0, 0];
    encoderPositions   = [0, 0, 0, 0];
    
    constructor(deviceID, deviceLink){
        this.deviceID = deviceID;
		this.deviceLink = deviceLink;

    }
    add_actuator(actuator, rotation, port){
        let error = false;
        
            if(port < 1 || port > 4){
                console.log("error: encoder port index out of bounds");
                error = true;
            }
        
            if(actuator < 1 || actuator > 4){
                console.log("error: encoder index out of bound!");
                error = true;
            }
            
            let j = 0;
            for(let i = 0; i < this.actuatorsActive; i++){
                if(this.motors[i].get_actuator() < actuator){
                    j++;
                }
                
                if(this.motors[i].get_actuator() == actuator){
                    console.log("error: actuator " + actuator + " has already been set");
                    error = true;
                }
            }
            
            if(!error){
                //let temp = new Actuator(this.actuatorsActive + 1);

                //const playerNames = ['name1', 'name2', 'name3'];
                //let players = [];
                //playerNames.forEach((playerName) => players.push(new Player(playerName)));

                let temp = [];
                for (var i = 0; i < this.actuatorsActive + 1; i++)
                {
                  temp.push(new Actuator(0));
                }

                //console.log(this.motors);
                //console.log(temp);
    
                this.arraycopy(this.motors, 0, temp, 0, this.motors.length);
                //this.arraycopy( src, src_pos, dst, dst_pos, length )
                //array1.copyWithin(0, 3, 4));
                
                if(j < this.actuatorsActive){
                  this.arraycopy(this.motors, j, temp, j+1, this.motors.length - j);
                }
                
                temp[j] = new Actuator(actuator, rotation, port);
                this.actuator_assignment(actuator, port);
                
                this.motors = temp;
                this.actuatorsActive++;
            }
        }
    
            add_encoder(encoder, rotation, offset, resolution, port){
                let error = false;
            
                if(port < 1 || port > 4){
                    console.log("error: encoder port index out of bounds");
                    error = true;
                }
                
                if(encoder < 1 || encoder > 4){
                    console.log("error: encoder index out of bound!");
                    error = true;
                }
                
                // determine index for copying
                let j = 0;
                for(let i = 0; i < this.encodersActive; i++){
                    if(this.encoders[i].get_encoder() < encoder){
                        j++;
                    }
                    
                    if(this.encoders[i].get_encoder() == encoder){
                        console.log("error: encoder " + encoder + " has already been set");
                        error = true;
                    }
                }
                
                if(!error){
                    //let temp = new Sensor[this.encodersActive + 1];
                    
                    let temp = [];
                    for (var i = 0; i < this.encodersActive + 1; i++)
                    {
                      temp.push(new Sensor(0));
                    }

                    this.arraycopy(this.encoders, 0, temp, 0, this.encoders.length);
              
                    if(j < this.encodersActive){
                      this.arraycopy(this.encoders, j, temp, j+1, this.encoders.length - j);
                    }
                    
                    temp[j] = new Sensor(encoder, rotation, offset, resolution, port);
                    this.encoder_assignment(encoder, port);
                    
                    this.encoders = temp;
                    this.encodersActive++;
                }
            }
    
                add_analog_sensor  (pin) {
                    // set sensor to be size zero
                    let error = false;
                    
                    let port = pin.charAt(0);
                    let number = pin.substring(1);
                    
                    let value = Integer.parseInt(number);
                    value = value + 54;
                    
                    for(let i = 0; i < this.sensorsActive; i++){
                        if(value == this.sensors[i].get_port()){
                            console.log("error: Analog pin: A" + (value - 54) + " has already been set");
                            error = true;
                        }
                    }
                    
                    if(port != 'A' || value < 54 || value > 65){
                            console.log("error: outside analog pin range");
                            error = true;
                    }
                    
                    if(!error){
                        let temp = Arrays.copyOf(this.sensors, this.sensors.length + 1);
                        temp[this.sensorsActive] = new Sensor();
                        temp[this.sensorsActive].set_port(value);
                        this.sensors = temp;
                        this.sensorsActive++;
                    }
                }
    
    add_pwm_pin(pin){
            
                    let error = false;
                    
                    for(let i = 0; i < this.pwmsActive; i++){
                        if(pin == this.pwms[i].get_pin()){
                            console.log("error: pwm pin: " + pin + " has already been set");
                            error = true;
                        }
                    }
                    
                    if(pin < 0 || pin > 13){
                            console.log("error: outside pwn pin range");
                            error = true;
                    }
            
                if(pin == 0 || pin == 1){
                    console.log("warning: 0 and 1 are not pwm pins on Haply M3 or Haply original");
                }
                    
                    
                    if(!error){
                        const temp = Arrays.copyOf(this.pwms, this.pwms.length + 1);
                        temp[this.pwmsActive] = new Pwm();
                        temp[this.pwmsActive].set_pin(pin);
                        this.pwms = temp;
                        this.pwmsActive++;
                    }
                }
    
    set_mechanism (mechanism){
        this.mechanism = mechanism;
    }
    
    device_set_parameters (){

        // initialize parameters
        console.log("initialize the board");
            
        this.communicationType = 1;
        
        let control;
        
        let encoderParameters = new Float32Array(); // was const
    
        let encoderParams = new Uint8Array;// was const
        let motorParams = new Uint8Array;// was const
        let sensorParams = new Uint8Array;// was const
        let pwmParams = new Uint8Array;// was const
        
        if(this.encodersActive > 0){	
      encoderParams = new Uint8Array(this.encodersActive + 1);
            control = 0;		
    
            for(let i = 0; i < this.encoders.length; i++){
                if(this.encoders[i].get_encoder() != (i+1)){
                    console.log("warning, improper encoder indexing");
                    this.encoders[i].set_encoder(i+1);
                    this.encoderPositions[this.encoders[i].get_port() - 1] = this.encoders[i].get_encoder();
                }
            }
            
            for(let i = 0; i < this.encoderPositions.length; i++){
                control = control >> 1;
                
                if(this.encoderPositions[i] > 0){
                    control = control | 0x0008;
                }
            }
            
            encoderParams[0] = control;
        
            encoderParameters = new Float32Array(2*this.encodersActive);
            
            let j = 0;
            for(let i = 0; i < this.encoderPositions.length; i++){
                if(this.encoderPositions[i] > 0){
                    encoderParameters[2*j] = this.encoders[this.encoderPositions[i]-1].get_offset(); 
                    encoderParameters[2*j+1] = this.encoders[this.encoderPositions[i]-1].get_resolution();
                    j++;
          encoderParams[j] = this.encoders[this.encoderPositions[i]-1].get_direction(); 
                }
            }
        }
        else{
      encoderParams = new Uint8Array(1);// byte[1];
      encoderParams[0] = 0;
            encoderParameters = new Float32Array(0);//float[0];
        }
        
        
        if(this.actuatorsActive > 0){
      motorParams = new Uint8Array(this.actuatorsActive + 1);//byte[this.actuatorsActive + 1];
            control = 0;
            
            for(let i = 0; i < this.motors.length; i++){
                if(this.motors[i].get_actuator() != (i+1)){
                    console.log("warning, improper actuator indexing");
                    this.motors[i].set_actuator(i+1);
                    this.actuatorPositions[this.motors[i].get_port() - 1] = this.motors[i].get_actuator();
                }
            }
            
            for(let i = 0; i < this.actuatorPositions.length; i++){
                control = control >> 1;
                
                if(this.actuatorPositions[i] > 0){
                    control = control | 0x0008;
                }
            }
            
            motorParams[0] = control;
      
      let j = 1;
      for(let i = 0; i < this.actuatorPositions.length; i++){
        if(this.actuatorPositions[i] > 0){
          motorParams[j] = this.motors[this.actuatorPositions[i]-1].get_direction();
          j++;
        }
      }
        }
    else{
      const motorParams = new Uint8Array(1);//[1];
      motorParams[0] = 0;
    }
        
        
        if(this.sensorsActive > 0){
            sensorParams = new Uint8Array[this.sensorsActive + 1];
            sensorParams[0] = this.sensorsActive;
            
            for(let i = 0; i < this.sensorsActive; i++){
                sensorParams[i+1] = this.sensors[i].get_port();
            }
            
            Arrays.sort(sensorParams);
            
            for(let i = 0; i < this.sensorsActive; i++){
                this.sensors[i].set_port(sensorParams[i+1]);
            }
            
        }
        else{
            sensorParams = new Uint8Array(1);//byte[1];
            sensorParams[0] = 0;
        }
    
    
    if(this.pwmsActive > 0){
      let temp = new Uint8Array(this.pwmsActive);
      
      pwmParams = new Uint8Array(this.pwmsActive + 1);
      pwmParams[0] = this.pwmsActive;
      
      
      for(let i = 0; i < this.pwmsActive; i++){
        temp[i] = this.pwms[i].get_pin();
      }
      
      Arrays.sort(temp);
      
      for(let i = 0; i < this.pwmsActive; i++){
        this.pwms[i].set_pin(temp[i]);
        pwmParams[i+1] = this.pwms[i].get_pin();
      }
      
    }
    else{
      pwmParams = new Uint8Array(1);//byte[1];
      pwmParams[0] = 0;
    }

        const encMtrSenPwm = new Uint8Array(motorParams.length  + encoderParams.length + sensorParams.length + pwmParams.length);
        this.arraycopy(motorParams, 0, encMtrSenPwm, 0, motorParams.length);
        this.arraycopy(encoderParams, 0, encMtrSenPwm, motorParams.length, encoderParams.length);
        this.arraycopy(sensorParams, 0, encMtrSenPwm, motorParams.length+encoderParams.length, sensorParams.length);
        this.arraycopy(pwmParams, 0, encMtrSenPwm, motorParams.length+encoderParams.length+sensorParams.length, pwmParams.length);
        
        console.log("sending the parameters");
        this.deviceLink.transmit(this.communicationType, this.deviceID, encMtrSenPwm, encoderParameters);	
    }

    actuator_assignment(actuator, port){
		if(this.actuatorPositions[port - 1] > 0){
			console.log("warning, double check actuator port usage");
		}
		
		this.actuatorPositions[port - 1] = actuator;
	}

    
    arraycopy(src, srcPos, dst, dstPos, length) {
        while (length--) dst[dstPos++] = src[srcPos++]; return dst;
    }
                
        
  //   // console.log(motorParams);
  //   // console.log(encMtrSenPwm);
  //   // console.log(motorParams.length);

  // //   console.log("src:" + src);
  // //   console.log(srcIndex);
  // //   console.log(dest);
  // //   console.log(destIndex);
  // //   console.log(length);
  // // dest.splice(destIndex, length, ...src.slice(srcIndex, srcIndex + length));
  // }
	 

 /**
  * assigns encoder positions based on actuator port
  */	
	encoder_assignment(encoder, port){
		
		if(this.encoderPositions[port - 1] > 0){
			console.log("warning, double check encoder port usage");
		}
		
		this.encoderPositions[port - 1] = encoder;
	}
    
    async device_read_data (){
        let communicationType = 2;
        let dataCount = 0;
        
        //float[] device_data = new float[sensorUse + encodersActive];
        console.log("starting device read")
        
        let device_data =  await this.deviceLink.receive()//communicationType, this.deviceID, this.sensorsActive + this.encodersActive);
        //this.deviceLink.receive(communicationType, this.deviceID, this.sensorsActive + this.encodersActive).then(device_data);
        console.log("the device data is: " + device_data);
        console.log(device_data[0]);
        console.log("done device read");
    
        for(let i = 0; i < this.sensorsActive; i++){
            this.sensors[i].set_value(device_data[dataCount]);
            dataCount++;
        }
        
        for(let i = 0; i < this.encoderPositions.length; i++){
            if(this.encoderPositions[i] > 0){
                this.encoders[this.encoderPositions[i]-1].set_value(device_data[dataCount]);
                dataCount++;
            }
        }
    }
    
    device_read_request (){
        let communicationType = 2;
        const pulses = new Uint8Array(pwmsActive);
        const encoderRequest = new Float32Array(actuatorsActive);
        
    for(let i = 0; i < pwms.length; i++){
      pulses[i] = pwms[i].get_value();
    }
    
        // think about this more encoder is detached from actuators
        let j = 0;
        for(let i = 0; i < this.actuatorPositions.length; i++){
            if(this.actuatorPositions[i] > 0){
                encoderRequest[j] = 0;
                j++;
            }
        }
        
        this.deviceLink.transmit(communicationType, this.deviceID, pulses, encoderRequest);
    }
    
    device_write_torques(){
        let communicationType = 2;
        const pulses = new Uint8Array(this.pwmsActive);
        const deviceTorques = new Float32Array(this.actuatorsActive);
        
    for(let i = 0; i < this.pwms.length; i++){
      pulses[i] = this.pwms[i].get_value();
    }
        
        let j = 0;
        for(let i = 0; i < this.actuatorPositions.length; i++){
            if(this.actuatorPositions[i] > 0){
                deviceTorques[j] = this.motors[this.actuatorPositions[i]-1].get_torque();
                j++;
            }
        }
        
        this.deviceLink.transmit(communicationType, this.deviceID, pulses, deviceTorques);
    }
    
    set_pwm_pulse (pin, pulse){
        
        for(let i = 0; i < this.pwms.length; i++){
          if(this.pwms[i].get_pin() == pin){
            this.pwms[i].set_pulse(pulse);
          }
        }
      }	
    
    get_pwm_pulse (pin){
       
        let pulse = 0;
        
        for(let i = 0; i < this.pwms.length; i++){
          if(this.pwms[i].get_pin() == pin){
            pulse = this.pwms[i].get_pulse();
          }
        }
        
        return pulse;
      }
    
    get_device_angles(){
        const angles = new Float32Array(this.encodersActive);
        
        for(let i = 0; i < this.encodersActive; i++){
            angles[i] = this.encoders[i].get_value();
        }
        
        return angles;
    }
    
    get_sensor_data (){
        const data = new Float32Array(this.sensorsActive);
        
        let j = 0;
        for(let i = 0; i < this.sensorsActive; i++){
            data[i] = this.sensors[i].get_value();
        }
    
        return data;
    }
    
    get_device_position (angles){
        this.mechanism.forwardKinematics(angles);
        var endEffectorPosition = this.mechanism.get_coordinate();
        
        return endEffectorPosition;
    }
    
    set_device_torques (forces){
        this.mechanism.torqueCalculation(forces);
        var torques = this.mechanism.get_torque();
        
        for(let i = 0; i < this.actuatorsActive; i++){
            this.motors[i].set_torque(torques[i]);
        }
        
        return torques;
    }
}

