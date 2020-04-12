let dt = 0.1;
let radar_x;
let radar_y;
let airplane_w;
let airplane_h;
let airplane_vel;
let noisyPath = [];
let correctedPath = [];
let slider;
class Airplane {
    constructor() {
        this.pos = 30;
        this.vel = airplane_vel;
        this.alt = height/2;
        this.state = [this.pos, this.vel, this.alt];
        this.w = airplane_w;
        this.h = airplane_h;
    }

    move() {
        this.alt = height/2;
        
        if(this.alt + this.h >= height - 10)
            this.alt = height - 10 - this.h;
        if(this.alt <= 10)
            this.alt = 10;
        this.pos += this.vel * dt;
        this.state = [this.pos, this.vel, this.alt];
    }

    draw() {

        fill(color(50,50,50));
        rect(this.pos, this.alt, this.w, this.h);
        noStroke();
    }
}

class Radar {
    constructor() {
        this.x = radar_x;
        this.y = radar_y;
    }

    draw() {
        fill(color(0,0,255));
        ellipse(this.x, this.y, 10, 10);
    }

    drawLines(plane) {
        stroke(color(0,0,0));
        line(this.x, this.y, plane.pos + plane.w/2, plane.alt + plane.h/2); // Slant
        line(this.x, this.y, plane.pos + plane.w / 2, this.y); // x
        line(plane.pos + plane.w / 2, this.y, plane.pos + plane.w / 2, plane.alt + plane.h / 2); // altitude
    }


}

let SIGMA_RANGE = 10;
class KalmanFilter {
    constructor(state) {
        this.state = state;
        this.R = math.matrix([[SIGMA_RANGE * SIGMA_RANGE]]);
        this.Q = math.matrix([[0.00001, 0, 0], [0, 0.000001, 0], [0, 0, 0.00001]]);
        this.P = math.matrix([[50, 0, 0], [0, 50, 0], [0, 0, 50]]);
        this.F = math.matrix([[1, dt, 0], [0, 1, 0], [0, 0, 1]]);
    }
    
    // Needed for Kalman filter
    // In all functions, the x param is the state
    // x[0] = plane distance from radar on OX
    // x[1] = plane velocity
    // x[2] = plane altitude

    draw(pos, alt, vel) {
        push();
        stroke(color(255,0,255));
        line(radar_x, radar_y, pos + airplane_w/2, alt+airplane_h/2);
        pop();
    
        push();
        stroke(color(0,255,0));
        line(radar_x, radar_y, this.state._data[0][0] + airplane_w/2, this.state._data[2][0] + airplane_h/2);
        pop();

        push();
        textSize(20);
        text('Noisy velocity: ', 10, 40);
        text(vel, 250, 40);
        text('Filtered velocity: ', 10, 70);
        text(this.state._data[1][0], 250, 70);

        text('Actual velocity: ', 10, 100);
        text(airplane_vel, 250, 100);


        pop();
    }

    hx() {
        // h(x) = sqrt(x^2 + y^2)
        return sqrt(this.state._data[0] * this.state._data[0] + this.state._data[2] * this.state._data[2]);
    }

    HJacobian() {
        //Jacobian of H evaluated for state input x
        let horiz_dist = this.state._data[0];
        let altitude = this.state._data[2];
        let denom = sqrt(horiz_dist * horiz_dist + altitude * altitude);

        return math.matrix([[horiz_dist/denom, 0., altitude/denom]]);
    }

    step(vel, alt, pos) {
        // Predict
        // x = Fx
        this.state = math.multiply(this.F, this.state);
        // P = FPFt + Q
        this.P = math.add(math.multiply(math.multiply(this.F, this.P), math.transpose(this.F)), this.Q);

        
        // Update step
        // Get Jacobian evaluation
        let H = this.HJacobian();
        // Get measurement with noise
        let _vel = vel + slider.value() * (Math.random() - 0.5);
        let _alt = alt + slider.value() * (Math.random() - 0.5);
        let _pos = pos + _vel * dt;
        let err = pos * 0.05 * (Math.random() - 0.5);
        let slant_dist = sqrt(_pos * _pos + _alt * _alt);
        let z = slant_dist + err; // Measurement
        // Calculate residual
        let y = math.matrix([[z - this.hx()]]);
        // Calculate Kalman Gain
        let K = math.multiply(this.P, math.transpose(H));
        let HPH = math.multiply(math.multiply(H, this.P), math.transpose(H));
        HPH = math.add(HPH, this.R);
        K = math.multiply(K, math.inv(HPH));

        // Update state
        this.state = math.add(this.state, math.multiply(K, y));
        // this.P = math.multiply(math.subtract(math.identity(3, 3), math.multiply(K, H)), this.P);
        
        let kh = math.subtract(math.identity(3, 3), math.multiply(K, H));
        this.P = math.multiply(math.multiply(kh, this.P), math.transpose(kh));
        this.P = math.add(this.P, math.multiply(math.multiply(K, this.R), math.transpose(K)));
        
        noisyPath.push([_pos, _alt]);
        correctedPath.push([this.state._data[0][0], this.state._data[2][0]]);

        this.draw(_pos, _alt, _vel);
    }
}

function drawPath(path, c) {
    for(let i = 0; i < path.length; i++) {
        push();
        stroke(c);
        strokeWeight(3);
        point(path[i][0] + airplane_w/2, path[i][1] + airplane_h/2);
        pop();
    }
}

let airplane;
let radar;
let kf;
function setup() {
    createCanvas(1500, 600);
    background(255);
    slider = createSlider(0, 100, 50, 1);
    slider.position(10, 120);
    radar_x = 10;
    radar_y = height - 10;
    airplane_w = 100;
    airplane_h = 50;
    airplane_vel = 5;
    airplane = new Airplane();
    radar = new Radar();
    kf = new KalmanFilter(math.matrix([[100], [100], [400]]))
}

function draw() {
    background(255);
    airplane.move();
    airplane.draw();

    // console.log(airplane.state)
    radar.draw();
    radar.drawLines(airplane);
    kf.step(airplane.vel, airplane.alt, airplane.pos);

    drawPath(noisyPath, color(255, 0, 255));
    drawPath(correctedPath, color(0, 255, 0));
}