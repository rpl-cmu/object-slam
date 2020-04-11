
class Pipeline:
    """Pipeline baseclass to be inherited for all tasks"""

    def __init__(self, source=None):
        """TODO: to be defined. """
        self.source = source

    def __iter__(self):
        return self.generator()

    def generator(self):
        """process and yields the next element in the iterator
        :returns: TODO

        """
        while True:
            try:
                data = next(self.source)
                if self.filter(data):
                    yield self.map(data)
            except StopIteration:
                return

    def __or__(self, other):
        """allows for pipelining tasks using | operator

        :object: pipeline object to be |'d
        :returns: TODO

        """
        if other is not None:
            # post pipelining
            other.source = self.generator()
            return other
        else:
            return self

    def filter(self, data):
        """TODO: Docstring for filter.

        :arg1: TODO
        :returns: TODO

        """
        return True

    def map(self, data):
        """overwrite to map the data

        :data: TODO
        :returns: TODO

        """
        return data


