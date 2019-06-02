#   -------------------------------------------------------------------------------
#   Description:
#   Function to generate a product order for testing purposes in the machine
#   assembly.
#   Also contains errorhandling for incorrect inputs
#   -------------------------------------------------------------------------------
#   Author: Sebastian Brink
#   Kandidatarbete COBOTS 2019
#   V.1.0.0
#   -------------------------------------------------------------------------------

import random


def generate_product_order(numberOfProducts: int, productList: list,
                           order='random'):
    ''' Generates a product order of specified size, products and order.
    Keyword arguments: \n
    numberOfProducts -- How many products are to be made
    productList -- List of product names to be made
    order --  What order you want the products to be made in \t
            random(default), alternating or selfdefined (list) \t
                e.g: [0, 1, 1, 0] for products of productList index 0 and 1
    '''
    generated_product_order = []
    for i in range(numberOfProducts):

        # Check if the order input is a list (e.g self defined order)
        if type(order) == list:
            if len(order) != numberOfProducts:

                # if the order length and number of products does not match
                # raise error
                raise ProductOrderSizeError(numberOfProducts, order)

            product = order[i]
            nextProduct = productList[product]

        elif str.lower(order) == 'random':
            nextProduct = productList[random.randrange(len(productList))]

        elif str.lower(order) == 'alternating':
            nextProduct = productList[i % 2]

        generated_product_order.append(nextProduct)

    return generated_product_order


class ProductOrderSizeError(Exception):
    def __init__(self, numberOfProducts, productOrder):
        self.number_of_products = numberOfProducts
        self.productOrder = productOrder

        if len(self.productOrder) < self.number_of_products:
            self.diffSym = '<'
        else:
            self.diffSym = '>'

    def __str__(self):
        return "The size of the product order does not match the " \
                "given numberOfProducts variable {} {} {}".format(
                    len(self.productOrder), self.diffSym,
                    self.number_of_products)
