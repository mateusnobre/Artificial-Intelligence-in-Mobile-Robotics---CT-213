import numpy as np
import random
from math import inf, floor


class Particle:
    """
    Represents a particle of the Particle Swarm Optimization algorithm.
    """
    def __init__(self, lower_bound, upper_bound):
        """
        Creates a particle of the Particle Swarm Optimization algorithm.

        :param lower_bound: lower bound of the particle position.
        :type lower_bound: numpy array.
        :param upper_bound: upper bound of the particle position.
        :type upper_bound: numpy array.
        """
        # Todo: implement
        delta = upper_bound - lower_bound
        self.x = np.random.uniform(lower_bound, upper_bound)
        self.v = np.random.uniform(-delta, delta)
        self.best_position = self.x
        self.best_value = -inf

class ParticleSwarmOptimization:
    """
    Represents the Particle Swarm Optimization algorithm.
    Hyperparameters:
        inertia_weight: inertia weight.
        cognitive_parameter: cognitive parameter.
        social_parameter: social parameter.
        num_particles: number of particles.
    :param hyperparams: hyperparameters used by Particle Swarm Optimization.
    :type hyperparams: Params.
    :param lower_bound: lower bound of particle position.
    :type lower_bound: numpy array.
    :param upper_bound: upper bound of particle position.
    :type upper_bound: numpy array.
    """
    def __init__(self, hyperparams, lower_bound, upper_bound):
        # Todo: implement
        self.i = -1 # indice to control generation's advance
        self.best_global = Particle(lower_bound,upper_bound)
        self.best_global.best_position = None
        self.lower_bound = lower_bound
        self.upper_bound = upper_bound
        self.delta = self.upper_bound - self.lower_bound
        self.w = hyperparams.inertia_weight
        self.phip = hyperparams.cognitive_parameter
        self.phig = hyperparams.social_parameter
        self.particles = []
        self.n_particles = hyperparams.num_particles
        for i in range(hyperparams.num_particles):
            self.particles.append(Particle(lower_bound, upper_bound))
    def get_best_position(self):
        """
        Obtains the best position so far found by the algorithm.

        :return: the best position.
        :rtype: numpy array.
        """
        # Todo: implement
        return self.best_global.best_position
    

    def get_best_value(self):
        """
        Obtains the value of the best position so far found by the algorithm.

        :return: value of the best position.
        :rtype: float.
        """
        # Todo: implement
        return self.best_global.best_value

    def get_position_to_evaluate(self):
        """
        Obtains a new position to evaluate.

        :return: position to evaluate.
        :rtype: numpy array.
        """
        # Todo: implement
        if self.i >= self.n_particles - 1:
            self.i = -1
            self.advance_generation()
        self.i += 1
        return self.particles[self.i].x

    def advance_generation(self):
        """
        Advances the generation of particles.
        """
        # Todo: implement
        rp = random.uniform(0,1)
        rg = random.uniform(0,1)
        for particle in self.particles:
            particle.v = self.w * particle.v + self.phip * rp * (particle.best_position - particle.x) + self.phig * rg * (self.best_global.best_position - particle.x)  
            particle.x += particle.v
            for i in range(3):
                particle.x[i] = min(max(particle.x[i], self.lower_bound[i]),self.upper_bound[i])
                particle.v[i] = min(max(particle.v[i],-self.delta[i]),self.delta[i])
            

    def notify_evaluation(self, value):
        """
        Notifies the algorithm that a particle position evaluation was completed.

        :param value: quality of the particle position.
        :type value: float.
        """
        # Todo: implement
        if value > self.particles[self.i].best_value:
            self.particles[self.i].best_value = value
            self.particles[self.i].best_position = np.array(self.particles[self.i].x)
        if value > self.best_global.best_value:
            self.best_global.best_position = np.array(self.particles[self.i].x)
            self.best_global.best_value = value
        
        