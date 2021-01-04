import numpy as np


class SimpleEvolutionStrategy:
    """
    Represents a simple evolution strategy optimization algorithm.
    The mean and covariance of a gaussian distribution are evolved at each generation.
    """
    def __init__(self, m0, C0, mu, population_size):
        """
        Constructs the simple evolution strategy algorithm.

        :param m0: initial mean of the gaussian distribution.
        :type m0: numpy array of floats.
        :param C0: initial covariance of the gaussian distribution.
        :type C0: numpy matrix of floats.
        :param mu: number of parents used to evolve the distribution.
        :type mu: int.
        :param population_size: number of samples at each generation.
        :type population_size: int.
        """
        self.m = m0
        self.C = C0
        self.mu = mu
        self.population_size = population_size
        self.samples = np.random.multivariate_normal(self.m, self.C, self.population_size)

    def ask(self):
        """
        Obtains the samples of this generation to be evaluated.
        The returned matrix has dimension (population_size, n), where n is the problem dimension.

        :return: samples to be evaluated.
        :rtype: numpy array of floats.
        """
        return self.samples

    def my_np_cov(self, parents, real_mean):
        C = np.zeros((2,2))
        m = np.full_like(parents, real_mean)
        parents = parents - m
        C = np.matmul(parents.T, parents)
        C = (1/len(parents)) * C
        return C
    
    def tell(self, fitnesses):
        """
        Tells the algorithm the evaluated fitnesses. The order of the fitnesses in this array
        must respect the order of the samples.

        :param fitnesses: array containing the value of fitness of each sample.
        :type fitnesses: numpy array of floats.
        """
        # Todo: implement this method
        indices = np.argsort(fitnesses)
       # indices = np.flip(indices)
        parents = []
        print('foiuf=nf==')
        for i in range(self.mu):
            print(fitnesses[i])
            parents.append(self.samples[indices[i]])
        self.m = np.mean(parents, axis = 0)
        self.C = self.my_np_cov(parents, np.mean(parents, axis = 0))
        self.samples = np.random.multivariate_normal(self.m, self.C, self.population_size)