#!/usr/bin/env python3
import rospy, time
from deap import base, creator, tools
import random
import datetime
from std_msgs.msg import Float64
from gazebo_msgs.msg import LinkStates
from geometry_msgs.msg import Pose, Twist
from std_srvs.srv import Empty

cart_pose = Pose()
pole_pose = Pose()
pole_twist = Twist()
y_angular = 0
cart_pose_x = 0
pole_tip_pose_z = 0

pub_cart = rospy.Publisher('/cart_controller/command', Float64, queue_size = 10)

creator.create("FitnessMulti", base.Fitness, weights=(-1.0, 1.0))  
creator.create("Individual", list, fitness=creator.FitnessMulti)

def get_cart_pose(data):
    global cart_pose, pole_twist, cart_pose_x, y_angular, pole_tip_pose_z
    ind = data.name.index('cart_pole::cart_link')
    cart_pose = data.pose[ind]

    ind_pitch = data.name.index('cart_pole::pole_link')
    pole_twist = data.twist[ind_pitch]

    ind_tip = data.name.index('cart_pole::tip_link')
    pole_tip_pose = data.pose[ind_tip]

    cart_pose_x = cart_pose.position.x
    y_angular = pole_twist.angular.y
    pole_tip_pose_z = pole_tip_pose.position.z

def gain_evaluation(individual):
    global y_angular, cart_pose_x, pole_tip_pose_z, pub_cart
    
    print(f"individual : {individual}")

    Kp_y = individual[0]
    Ki_y = individual[1]
    Kd_y = individual[2]
    Kp_p = individual[3]
    Ki_p = individual[4]
    Kd_p = individual[5]
    time_interval = 0.005

    rospy.wait_for_service('/gazebo/reset_simulation')
    reset_simulation_client = rospy.ServiceProxy('/gazebo/reset_simulation',Empty) #rosservice call gazebo/pause_physics
    reset_simulation_client()

    yaw_angle = 0
    target_yaw_angle = 0
    target_cart_pose_x = 0
    last_error_yaw = 0
    last_error_pos = 0
    integral_position_error = 0
    integral_yaw_error = 0
    pole_tip_height_sum = 0
    x_displacement_sum = 0

    for i in range(1000):
        time1 = time.time()
        pole_tip_height_sum += pole_tip_pose_z
        yaw_angle += y_angular*time_interval
        error_yaw  = target_yaw_angle - yaw_angle
        integral_yaw_error += (error_yaw + last_error_yaw)*time_interval/2
        effort_yaw = -(Kp_y*error_yaw  + 
                       Ki_y*integral_yaw_error +
                       Kd_y*(error_yaw - last_error_yaw)/time_interval)

        error_pos = target_cart_pose_x - cart_pose_x
        integral_position_error += (error_pos + last_error_pos)*time_interval/2
        x_displacement_sum += abs(cart_pose_x)
        effort_pos = -(Kp_p*error_pos  + 
                       Ki_p*integral_position_error +
                       Kd_p*(error_pos - last_error_pos)/time_interval)   

        effort = effort_yaw + effort_pos    

        last_error_yaw = error_yaw
        last_error_pos = error_pos

        pub_cart.publish(effort)

        time2 = time.time()
        interval = time2 - time1
        if(interval < time_interval):
            time.sleep(time_interval - interval)
    
    integral_position_error = 0
    integral_yaw_error = 0

    print(f"x_displacement_sum : {x_displacement_sum}, pole_tip_height_sum : {pole_tip_height_sum}\n")  

    return x_displacement_sum, pole_tip_height_sum

if __name__ == '__main__':
    rospy.init_node('pid_gain_optimizer', anonymous=True)
    rospy.Subscriber("/gazebo/link_states", LinkStates, get_cart_pose)

    print("#####genetic optimization#####")
    toolbox = base.Toolbox()
    toolbox.register("attr_gene", random.uniform , 0, 20)
    toolbox.register("evaluate", gain_evaluation)
    toolbox.register("mate", tools.cxBlend, alpha=0.2)
    toolbox.register("select", tools.selTournament, tournsize=3)

    # GA parameters
    N_GEN = 6        # number of generations
    POP_SIZE = 60    # number of individuals
    CX_PB = 0.5      # crossover probability
    MUT_PB = 0.3     # mutation probability

    random.seed(datetime.datetime.now())

    points_number = 6
    toolbox.register("mutate", tools.mutGaussian, mu=0, sigma=0.5, indpb=0.2)
    toolbox.register("individual", tools.initRepeat, creator.Individual, toolbox.attr_gene, points_number) #toolbox.attr_gene
    toolbox.register("population", tools.initRepeat, list, toolbox.individual)

    # creating population
    pop = toolbox.population(n=POP_SIZE)

    # evaluating each individual
    fitnesses = list(map(toolbox.evaluate, pop))
    for ind, fit in zip(pop, fitnesses):
        ind.fitness.values = fit
    print("  Evaluated %i individuals" % len(pop))

    # fitness of each individual
    fits = [ind.fitness.values[0] for ind in pop]

    g = 0
    while g < N_GEN:
        g = g + 1
        print("-- Generation %i --" % g)

        offspring = toolbox.select(pop, len(pop))
        offspring = list(map(toolbox.clone, offspring))

        for child1, child2 in zip(offspring[::2], offspring[1::2]):
            if random.random() < CX_PB:
                toolbox.mate(child1, child2)

                del child1.fitness.values
                del child2.fitness.values

        for mutant in offspring:
            if random.random() < MUT_PB:
                toolbox.mutate(mutant)

                del mutant.fitness.values

        invalid_ind = []
        for ind in offspring:
            if not ind.fitness.valid:
                invalid_ind.append(ind)

        fitnesses = map(toolbox.evaluate, invalid_ind)
        for ind, fit in zip(invalid_ind, fitnesses):
            ind.fitness.values = fit

        pop[:] = offspring
        
        fits0 = [ind.fitness.values[0] for ind in pop]

        length = len(pop)
        mean = sum(fits0) / length
        sum2 = sum(x * x for x in fits0)
        std = abs(sum2 / length - mean**2)**0.5

        print("  Min %s" % min(fits0))
        print("  Max %s" % max(fits0))
        print("  Avg %s" % mean)
        print("  Std %s" % std)

    # choosing the best individual
    best_ind = tools.selBest(pop, 1)[0]
    print("Best individual is %s, %s" % (best_ind, best_ind.fitness.values))