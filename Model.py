import parl
from parl import layers

# actor
class Actor(parl.Model):
    def __init__(self, act_dim):
        self.fc1 = layers.fc(size=16, act="relu")
        self.fc2 = layers.fc(size=32, act="relu")
        self.fc3 = layers.fc(size=64, act="relu")
        self.fc4 = layers.fc(size=32, act="relu")
        self.fc5 = layers.fc(size=act_dim, act="tanh") 

    def policy(self, obs):
        out = self.fc1(obs)
        out = self.fc2(out)
        out = self.fc3(out)
        out = self.fc4(out)
        out = self.fc5(out)
        return out

# critic
class Critic(parl.Model):
    def __init__(self):
        self.fc1 = layers.fc(size=16, act="relu")
        self.fc2 = layers.fc(size=32, act="relu")
        self.fc3 = layers.fc(size=64, act="relu")
        self.fc4 = layers.fc(size=32, act="relu")
        self.fc5 = layers.fc(size=1, act=None) 

    def value(self, obs, act):
        concat = layers.concat([obs, act], axis=1)
        out = self.fc1(concat)
        out = self.fc2(out)
        out = self.fc3(out)
        out = self.fc4(out)
        Q = self.fc5(out)
        Q = layers.squeeze(Q, axes=[1])
        return Q

# integate actor net and critic net together
class Model(parl.Model):
    def __init__(self, act_dim):
        self.actor_model = Actor(act_dim)
        self.critic_model = Critic()

    def policy(self, obs):
        return self.actor_model.policy(obs)

    def value(self, obs, act):
        return self.critic_model.value(obs, act)

    # get actor's parameter
    def get_actor_params(self):
        return self.actor_model.parameters()