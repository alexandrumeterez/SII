let ROBOT_W = 150;
let ROBOT_H = 80;
let WHEEL_W = 40;
let WHEEL_H = 10;
let dt = 0.1;
class Robot {
    constructor() {
        this.x = width/2;
        this.y = height/2;
        this.theta = 0;

        this.vel = 1;
        this.angularVel = 2;
        this.steering = 1;
    }

    move() {
        if(this.steering > 0) {
            let turning_radius = ROBOT_W / sin(radians(this.steering));
            this.angularVel = this.vel / turning_radius;
        } else {
            this.angularVel = 0;
        }


        this.x += this.vel * dt;
        this.y += this.vel * dt;

        this.theta += degrees(this.angularVel) * dt;

    }

    draw() {
        rectMode(CENTER);

        push();
        // Origin
        strokeWeight(5);
        fill(0);
        point(this.x, this.y);
        pop();

        push();
        // Body
        translate(this.x, this.y);
        rotate(this.theta);
        translate(-this.x, -this.y);
        fill(color(255, 0, 0, 0));
        rect(this.x, this.y, ROBOT_W, ROBOT_H);
        pop();

        // Rear left wheel
        push();
        translate(this.x, this.y);
        rotate(this.theta);
        translate(-this.x, -this.y);
        rect(this.x - ROBOT_W / 2, this.y - ROBOT_H / 2, WHEEL_W, WHEEL_H);
        
        pop();
        
        // Rear right wheel
        translate(this.x, this.y);
        rotate(this.theta);
        translate(-this.x, -this.y);
        push();
        rect(this.x - ROBOT_W / 2, this.y + ROBOT_H / 2, WHEEL_W, WHEEL_H);
        pop();

        // Front left wheel
        push();
        translate(this.x, this.y);
        rotate(this.theta);
        translate(-this.x, -this.y);
        rect(this.x + ROBOT_W / 2, this.y - ROBOT_H / 2, WHEEL_W, WHEEL_H);
        pop();

        // Front right wheel
        push();
        translate(this.x, this.y);
        rotate(this.theta);
        translate(-this.x, -this.y);
        rect(this.x + ROBOT_W / 2, this.y + ROBOT_H / 2, WHEEL_W, WHEEL_H);
        pop();
    }
}
let robot;

function setup() {
    createCanvas(800, 600);
    background(255);

    robot = new Robot();
}

function draw() {
    background(255);
    
    robot.move();
    robot.draw();
}